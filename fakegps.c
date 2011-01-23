/*
**
** Copyright (C) 2010, 2011 The NitDroid Project
** Copyright (C) 2008 The Android Open Source Project
**
** Author: Alexey Roslyakov <alexey.roslyakov@newsycat.com>
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>

#define  LOG_TAG  "fakegps"
#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};


/* this is the state of our connection to the qemu_gpsd daemon */
typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            callbacks;
    pthread_t               thread;
    int                     control[2];
} GpsState;

static GpsState  _gps_state[1];

static GpsLocation fix = {
    .size = sizeof(GpsLocation),
    .accuracy = 5,
};

static float getFloatProperty(const char *propName)
{
    float res = 0.0f;
    char propValue[PROPERTY_VALUE_MAX];
    if (property_get(propName, propValue, NULL))
        sscanf(propValue, "%f", &res);

    return res;
}

static inline void updateFix()
{
    fix.latitude = getFloatProperty("hw.fakegps.latitude");
    fix.longitude = getFloatProperty("hw.fakegps.longitude");
    fix.altitude = getFloatProperty("hw.fakegps.altitude");
    LOGD("latitude=%f, longitude=%f, altitude=%f",
         fix.latitude,
         fix.longitude,
         fix.altitude);

    fix.flags = GPS_LOCATION_HAS_LAT_LONG | GPS_LOCATION_HAS_ACCURACY | GPS_LOCATION_HAS_SPEED;
    fix.flags |= ( fix.altitude == 0.0f ? 0 : GPS_LOCATION_HAS_ALTITUDE);
}

static void
gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    write( s->control[0], &cmd, 1 );
    pthread_join(s->thread, &dummy);

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // close connection to the QEMU GPS daemon
    close( s->fd ); s->fd = -1;
    s->init = 0;
}


static void
gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        LOGD("%s: could not send CMD_START command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}


static void
gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        LOGD("%s: could not send CMD_STOP command: ret=%d: %s",
             __FUNCTION__, ret, strerror(errno));
}


static int
epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}


static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static void
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];

    // register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    //epoll_register( epoll_fd, gps_fd );

    LOGD("gps thread running");

    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_fd, events, 2, -1 );
        if (nevents < 0) {
            if (errno != EINTR)
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
        LOGD("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    LOGD("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        LOGD("gps thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START) {
                        if (!started) {
                            LOGD("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
                            updateFix();
                            state->callbacks.location_cb(&fix);
                        }
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            LOGD("gps thread stopping");
                            started = 0;
                        }
                    }
                }
                else if (fd == gps_fd)
                {
                    char  buff[32];
                    LOGD("gps fd event");
                    for (;;) {
                        int  nn, ret;

                        ret = read( fd, buff, sizeof(buff) );
                        if (ret < 0) {
                            if (errno == EINTR)
                                continue;
                            if (errno != EWOULDBLOCK)
                                LOGE("error while reading from gps daemon socket: %s:", strerror(errno));
                            break;
                        }
                        LOGD("received %d bytes: %.*s", ret, ret, buff);
                    }
                    LOGD("gps fd event end");
                }
                else
                {
                    LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
Exit:
    {} // nothing
}


static void
gps_state_init( GpsState*  state )
{
    state->init       = 1;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

    state->thread = state->callbacks.create_thread_cb("fakegps_cb", gps_state_thread, state);
    if (!state->thread) {
        LOGE("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }

    LOGD("gps state initialized");
    return;

Fail:
    gps_state_done( state );
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/


static int
fake_gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;
    s->callbacks = *callbacks;

    if (!s->init)
        gps_state_init(s);

    return 0;
}

static void
fake_gps_cleanup(void)
{
    GpsState*  s = _gps_state;

    if (s->init)
        gps_state_done(s);
}


static int
fake_gps_start()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        LOGD("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    LOGD("%s: called", __FUNCTION__);
    gps_state_start(s);
    return 0;
}


static int
fake_gps_stop()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        LOGD("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    LOGD("%s: called", __FUNCTION__);
    gps_state_stop(s);
    return 0;
}


static int
fake_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

static int
fake_gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}

static void
fake_gps_delete_aiding_data(GpsAidingData flags)
{
}

static int fake_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
                                      uint32_t min_interval, uint32_t preferred_accuracy,
                                      uint32_t preferred_time)
{
    // FIXME - support fix_frequency
    LOGD("%s(mode=%d, min_interval=%u)", __FUNCTION__, mode, min_interval);
    return 0;
}

static const void*
fake_gps_get_extension(const char* name)
{
    return NULL;
}

static const GpsInterface fakeGpsInterface = {
    .size = sizeof(GpsInterface),
    .init = fake_gps_init,
    .start = fake_gps_start,
    .stop = fake_gps_stop,
    .cleanup = fake_gps_cleanup,
    .inject_time = fake_gps_inject_time,
    .inject_location = fake_gps_inject_location,
    .delete_aiding_data = fake_gps_delete_aiding_data,
    .set_position_mode = fake_gps_set_position_mode,
    .get_extension = fake_gps_get_extension,
};

static const GpsInterface* gps__get_gps_interface()
{
    LOGD("%s", __FUNCTION__);
    return &fakeGpsInterface;
}

static int open_gps(const struct hw_module_t* module, char const* name,
                    struct hw_device_t** device)
{
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    dev->get_gps_interface = gps__get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}

static struct hw_module_methods_t fakegps_module_methods = {
    .open = open_gps
};

/*
 * The GPS Hardware Module
 */
struct hw_module_t HAL_MODULE_INFO_SYM =
{
    .tag           = HARDWARE_MODULE_TAG,
    .version_major = 2,
    .version_minor = 0,
    .id            = GPS_HARDWARE_MODULE_ID,
    .name          = "FakeGPS module",
    .author        = "Alexey Roslyakov<alexey.roslyakov@newsycat.com>",
    .methods       = &fakegps_module_methods,
};
