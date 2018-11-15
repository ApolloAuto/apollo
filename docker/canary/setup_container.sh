#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

USER_NAME=apollo
USER_GROUP=apollo

# Align group and user id as the host user, so the permission is seamless.
usermod -u ${DOCKER_USER_ID} ${USER_NAME}
groupmod -g ${DOCKER_GRP_ID} ${USER_GROUP}

# setup GPS device
ls /dev/novatel? > /dev/null
if [ $? -eq 0 ]; then
  chmod a+rw /dev/novatel?
fi

# setup camera device
if [ -e /dev/camera/obstacle ]; then
  chmod a+rw /dev/camera/obstacle
fi
if [ -e /dev/camera/trafficlights ]; then
  chmod a+rw /dev/camera/trafficlights
fi

bash
