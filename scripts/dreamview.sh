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


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start_fe() {
  /apollo/bazel-bin/modules/dreamview/dreamview \
    --flagfile=/apollo/modules/common/data/global_flagfile.txt
}

function start() {
  LOG="${APOLLO_ROOT_DIR}/data/log/dreamview.out"
  nohup /apollo/bazel-bin/modules/dreamview/dreamview \
    --flagfile=/apollo/modules/common/data/global_flagfile.txt > ${LOG} 2>&1 &
}

function stop() {
  killall -9 /apollo/bazel-bin/modules/dreamview/dreamview
}

case $1 in
  start_fe)
    start_fe
    ;;
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    start
    ;;
esac
