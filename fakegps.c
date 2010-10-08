/*
**
** Copyright (C) 2010 The NitDroid Project
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
#include <hardware_legacy/gps.h>


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
static void*
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    //NmeaReader  reader[1];
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];

    //nmea_reader_init( reader );

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
                            state->callbacks.location_cb(&fix);
                            //nmea_reader_set_callback( reader, state->callbacks.location_cb );
                        }
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            LOGD("gps thread stopping");
                            started = 0;
                            //nmea_reader_set_callback( reader, NULL );
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
                        //for (nn = 0; nn < ret; nn++)
                        //nmea_reader_addc( reader, buff[nn] );
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
    return NULL;
}


static void
gps_state_init( GpsState*  state )
{
    state->init       = 1;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;

    updateFix();
    
#if 0
    state->fd = open("/dev/null", O_RDONLY );

    if (state->fd < 0) {
        LOGD("no gps emulation detected");
        return;
    }

    LOGD("gps emulation will read from '%s' qemud channel", "fake" );
#endif

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

    if ( pthread_create( &state->thread, NULL, gps_state_thread, state ) != 0 ) {
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

    if (!s->init)
        gps_state_init(s);

#if 0
    if (s->fd < 0)
        return -1;
#endif

    s->callbacks = *callbacks;

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

static int fake_gps_set_position_mode(GpsPositionMode mode, int fix_frequency)
{
    // FIXME - support fix_frequency
    LOGD("%s(mode=%d, fix_frequency=%d)", __FUNCTION__, mode, fix_frequency);
    return 0;
}

static const void*
fake_gps_get_extension(const char* name)
{
    return NULL;
}

static const GpsInterface  fakeGpsInterface = {
    fake_gps_init,
    fake_gps_start,
    fake_gps_stop,
    fake_gps_cleanup,
    fake_gps_inject_time,
    fake_gps_inject_location,
    fake_gps_delete_aiding_data,
    fake_gps_set_position_mode,
    fake_gps_get_extension,
};

const GpsInterface* gps_get_hardware_interface()
{
    LOGD("%s", __FUNCTION__);
#if 1
    return &fakeGpsInterface;
#else
    return 0;
#endif
}
