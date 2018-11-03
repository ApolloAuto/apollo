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

function setup() {
  bash scripts/canbus.sh start
  bash scripts/gps.sh start
  bash scripts/localization.sh start
  bash scripts/control.sh start
}

function start() {
  TIME=`date +%F_%H_%M`
  if [ -e data/log/garage.csv ]; then
    cp data/log/garage.csv data/log/garage-${TIME}.csv
  fi

  NUM_PROCESSES="$(pgrep -c -f "record_play/rtk_recorder.py")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    python modules/tools/record_play/rtk_recorder.py
  fi
}

function stop() {
  pkill -SIGKILL -f rtk_recorder.py
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
  *)
    start
    ;;
esac
