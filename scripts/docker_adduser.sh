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
adduser --disabled-password --force-badname --gecos '' "$DOCKER_USER" \
    --uid "$DOCKER_USER_ID" --gid "$DOCKER_GRP_ID" 2>/dev/null
usermod -aG sudo "$DOCKER_USER"
echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
cp -r /etc/skel/. /home/${DOCKER_USER}
echo 'if [ -e "/apollo/scripts/apollo_base.sh" ]; then source /apollo/scripts/apollo_base.sh; fi' >> "/home/${DOCKER_USER}/.bashrc"
echo "ulimit -c unlimited" >> /home/${DOCKER_USER}/.bashrc

chown -R ${DOCKER_USER}:${DOCKER_GRP} "/home/${DOCKER_USER}"

# setup GPS device
if [ -e /dev/novatel0 ]; then
  chmod a+rw /dev/novatel0
fi
if [ -e /dev/novatel1 ]; then
  chmod a+rw /dev/novatel1
fi
if [ -e /dev/novatel2 ]; then
  chmod a+rw /dev/novatel2
fi

# setup camera device
if [ -e /dev/camera/obstacle ]; then
  chmod a+rw /dev/camera/obstacle
fi
if [ -e /dev/camera/trafficlights ]; then
  chmod a+rw /dev/camera/trafficlights
fi


if [ "$RELEASE_DOCKER" != "1" ];then
  # setup map data
  if [ -e /home/tmp/modules_data ]; then
    cp -r /home/tmp/modules_data/* /apollo/modules/
    chown -R ${DOCKER_USER}:${DOCKER_GRP} "/apollo/modules"
  fi
  # setup ros package
  # this is a temporary solution to avoid ros package downloading.
  ROS="/home/tmp/ros"
  chmod a+w "${ROS}/share/velodyne/launch/start_velodyne.launch"
  chmod a+w -R "${ROS}/share/velodyne_pointcloud/params"
  chmod a+w "${ROS}/share/gnss_driver/launch/gnss_driver.launch"
  chmod a+w "${ROS}/share/gnss_driver/conf/gnss_conf_mkz.txt"
fi
