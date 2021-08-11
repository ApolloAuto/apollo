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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"

function setup() {
  bash ${TOP_DIR}/scripts/canbus.sh start
  bash ${TOP_DIR}/scripts/gps.sh start
  bash ${TOP_DIR}/scripts/localization.sh start
  bash ${TOP_DIR}/scripts/control.sh start
}

function start() {
  TIME="$(date +%F_%H_%M)"
  if [ -f ${TOP_DIR}/data/log/garage.csv ]; then
    cp ${TOP_DIR}/data/log/garage.csv ${TOP_DIR}/data/log/garage-${TIME}.csv
  fi

  NUM_PROCESSES="$(pgrep -f "record_play/rtk_recorder" | grep -cv '^1$')"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    ${TOP_DIR}/bazel-bin/modules/tools/record_play/rtk_recorder
  fi
}

function stop() {
  pkill -SIGKILL -f rtk_recorder
}

case $1 in
  setup)
    setup
    ;;
  start)
    start
    ;;
  stop)
    stop
    ;;
  restart)
    stop
    start
    ;;
  *)
    start
    ;;
esac
