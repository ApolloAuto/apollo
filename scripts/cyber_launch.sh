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


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start() {
  LAUNCH_FILE=$1
  LOG="/apollo/data/log/$(basename ${LAUNCH_FILE}).start.log"
  nohup cyber_launch start "${LAUNCH_FILE}" </dev/null >"${LOG}" 2>&1 &
}

function stop() {
  LAUNCH_FILE=$1
  LOG="/apollo/data/log/$(basename ${LAUNCH_FILE}).stop.log"
  nohup cyber_launch stop "${LAUNCH_FILE}" < /dev/null >"${LOG}" 2>&1 &
}

# run command_name launch_file.
function run() {
  case $1 in
    start)
      shift
      start $@
      ;;
    stop)
      shift
      stop $@
      ;;
    *)
      echo "Unknow command: $1"
      exit 1
      ;;
  esac
}

run "$@"
