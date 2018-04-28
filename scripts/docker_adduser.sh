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

USER=apollo
GROUP=apollo

# TODO(xiaoxq): It's in migration.
# Currently we support either existing 'apollo' user or adding on the fly. In
# the near future we'll add 'apollo' user directly in image.
# In either situation, we set the GROUP ID and USER ID to be the same with the
# docker user, so all the permissions are consistent.
getent passwd ${USER} > /dev/null
if [ $? -eq 0 ]; then
  # User exists, update its ID.
  usermod -u ${DOCKER_USER_ID} ${USER}
  groupmod -g ${DOCKER_GRP_ID} ${GROUP}
else
  # Add new user.
  addgroup --gid "${DOCKER_GRP_ID}" "${GROUP}"
  adduser --disabled-password --force-badname --gecos '' "${USER}" \
      --uid "${DOCKER_USER_ID}" --gid "${DOCKER_GRP_ID}" 2>/dev/null
  usermod -aG sudo "${USER}"
fi

echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
cp -r /etc/skel/. /home/${USER}
echo "export PATH=/apollo/scripts:$PATH" >> /home/${USER}/.bashrc
echo 'if [ -e "/apollo/scripts/apollo_base.sh" ]; then source /apollo/scripts/apollo_base.sh; fi' >> "/home/${USER}/.bashrc"
echo "ulimit -c unlimited" >> /home/${USER}/.bashrc

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
    chown -R ${USER}:${GROUP} "/apollo/modules"
  fi

  # setup ros package
  # this is a temporary solution to avoid ros package downloading.
  ROS="/home/tmp/ros"
  chmod a+w "${ROS}/share/velodyne/launch/start_velodyne.launch"
  chmod a+w -R "${ROS}/share/velodyne_pointcloud/params"
  chmod a+w "${ROS}/share/gnss_driver/launch/gnss_driver.launch"
  chmod a+w "${ROS}/share/gnss_driver/conf/gnss_conf_mkz.txt"
fi
