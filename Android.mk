#
# Copyright (C) 2010 The NITDroid Project
# Copyright (C) 2008 The Android Open Source Project
#
# Contact: Alexey Roslyakov <alexey.roslyakov@newsycat.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

ifeq ($(BOARD_HAVE_FAKE_GPS),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# HAL module implemenation, not prelinked and stored in
# hw/<OVERLAY_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE := gps.$(PRODUCT_BRAND)
LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := \
	libcutils
##

LOCAL_SRC_FILES += \
  fakegps.c
##

LOCAL_CFLAGS += \
# include any needed compile flags

LOCAL_C_INCLUDES:= \
# include any needed local header files

include $(BUILD_SHARED_LIBRARY)

endif
