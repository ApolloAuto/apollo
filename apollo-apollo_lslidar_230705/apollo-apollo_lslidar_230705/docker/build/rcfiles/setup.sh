#! /usr/bin/env bash
###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

function pathremove() {
  local IFS=':'
  local NEWPATH
  local DIR
  local PATHVARIABLE=${2:-PATH}
  for DIR in ${!PATHVARIABLE}; do
    if [ "$DIR" != "$1" ]; then
      NEWPATH=${NEWPATH:+$NEWPATH:}$DIR
    fi
  done
  export $PATHVARIABLE="$NEWPATH"
}

function pathprepend() {
  pathremove $1 $2
  local PATHVARIABLE=${2:-PATH}
  export $PATHVARIABLE="$1${!PATHVARIABLE:+:${!PATHVARIABLE}}"
}

function pathappend() {
  pathremove $1 $2
  local PATHVARIABLE=${2:-PATH}
  export $PATHVARIABLE="${!PATHVARIABLE:+${!PATHVARIABLE}:}$1"
}

function setup_gpu_support() {
  if [ -e /usr/local/cuda/ ]; then
    pathprepend /usr/local/cuda/bin
  fi
}

if [ ! -f /apollo/LICENSE ]; then

  APOLLO_IN_DOCKER=false
  APOLLO_PATH="/opt/apollo/neo"
  APOLLO_ROOT_DIR=${APOLLO_PATH}/packages
  
  if [ -f /.dockerenv ]; then
    APOLLO_IN_DOCKER=true
  fi

  export APOLLO_PATH
  export APOLLO_ROOT_DIR=${APOLLO_PATH}/packages
  export CYBER_PATH=${APOLLO_ROOT_DIR}/cyber
  export APOLLO_IN_DOCKER
  export APOLLO_SYSROOT_DIR=/opt/apollo/sysroot
  export CYBER_DOMAIN_ID=80
  export CYBER_IP=127.0.0.1
  export GLOG_log_dir=${APOLLO_PATH}/data/log
  export GLOG_alsologtostderr=0
  export GLOG_colorlogtostderr=1
  export GLOG_minloglevel=0
  export sysmo_start=0
  export USE_ESD_CAN=false

fi


pathprepend /opt/apollo/neo/bin
setup_gpu_support
