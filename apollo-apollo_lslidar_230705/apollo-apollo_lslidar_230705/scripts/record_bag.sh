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

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${DIR}/apollo_base.sh"

function start() {
  decide_task_dir $@
  cd "${TASK_DIR}"

  # Start recording.
  record_bag_env_log
  LOG="/tmp/apollo_record.out"
  NUM_PROCESSES="$(pgrep -f "cyber_recorder record" | grep -cv '^1$')"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    nohup cyber_recorder record -a -i 60 -m 2048 < /dev/null > "${LOG}" 2>&1 &
  fi
}

function stop() {
  pkill -SIGINT -f record
}

function help() {
  echo "Usage:"
  echo "$0 [start]                     Record bag to data/bag."
  echo "$0 stop                        Stop recording."
  echo "$0 help                        Show this help message."
}

case $1 in
  start)
    shift
    start $@
    ;;
  stop)
    shift
    stop $@
    ;;
  help)
    shift
    help $@
    ;;
  restart)
    shift
    stop $@
    start $@
    ;;
  *)
    start $@
    ;;
esac
