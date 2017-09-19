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


if  pidof -o %PPID -x "roscore" > /dev/null; then

  if rosnode list |grep velodyne64_driver > /dev/null; then
    rosnode kill /velodyne64_driver
  fi

  if rosnode list |grep velodyne64_convert > /dev/null; then
    rosnode kill /velodyne64_convert
  fi

  if rosnode list |grep velodyne64_compensator > /dev/null; then
    rosnode kill /velodyne64_compensator
  fi

  if rosnode list |grep velodyne_nodelet_manager > /dev/null; then
    rosnode kill /velodyne_nodelet_manager
  fi

fi

roslaunch velodyne start_velodyne.launch
