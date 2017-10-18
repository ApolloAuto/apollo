#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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


addgroup --gid "$DOCKER_GRP_ID" "$DOCKER_GRP"
adduser --disabled-password --gecos '' "$DOCKER_USER" \
    --uid "$DOCKER_USER_ID" --gid "$DOCKER_GRP_ID" 2>/dev/null
usermod -aG sudo "$DOCKER_USER"
echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
cp -r /etc/skel/. /home/${DOCKER_USER}
echo 'if [ -e "/apollo/scripts/apollo_base.sh" ]; then source /apollo/scripts/apollo_base.sh; fi' >> "/home/${DOCKER_USER}/.bashrc"
echo "ulimit -c unlimited" >> /home/${DOCKER_USER}/.bashrc

chown -R ${DOCKER_USER}:${DOCKER_GRP} "/home/${DOCKER_USER}"

# grant caros user to access GPS device
if [ -e /dev/ttyUSB0 ]; then
    sudo chmod a+rw /dev/ttyUSB0 /dev/ttyUSB1
fi

MACHINE_ARCH=$(uname -m)
ROS_TAR="ros-indigo-apollo-1.5.0-${MACHINE_ARCH}.tar.gz"
if [ "$RELEASE_DOCKER" != "1" ];then
  # setup map data
  if [ -e /home/tmp/modules_data ]; then
    cp -r /home/tmp/modules_data/* /apollo/modules/
    chown -R ${DOCKER_USER}:${DOCKER_GRP} "/apollo/modules"
  fi

  # setup ros package
  # this is a tempary solution to avoid ros package downloading.
  MD5="$(echo -n '/apollo' | md5sum | cut -d' ' -f1)"
  EXTERNAL="/home/${DOCKER_USER}/.cache/bazel/_bazel_${USER}/${MD5}/external"
  if [ ! -e "$EXTERNAL" ];then
    mkdir -p $EXTERNAL
    chown -R ${DOCKER_USER}:${DOCKER_GRP} "${EXTERNAL}"
  fi
  ROS="${EXTERNAL}/ros"
  if [ -e "$ROS" ]; then
    rm -rf $ROS
  fi
  tar xzf "/home/tmp/${ROS_TAR}" -C $EXTERNAL
  cd $ROS
  ln -s /apollo/third_party/ros.BUILD BUILD.bazel
  chown -R ${DOCKER_USER}:${DOCKER_GRP} "${ROS}"
fi
