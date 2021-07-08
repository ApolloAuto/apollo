#!/usr/bin/env bash

###############################################################################
# Copyright 2021 The Apollo Authors. All Rights Reserved.
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
cd "${TOP_DIR}"
source "${TOP_DIR}/scripts/apollo_base.sh"

# Docker environment
DREAMVIEW_PORT=8888
DREAMVIEW_URL="http://localhost:${DREAMVIEW_PORT}"

# Host environment
HOST_IP="${HOST_IP:-localhost}"
HOST_DREAMVIEW_PORT="${HOST_DREAMVIEW_PORT:-$DREAMVIEW_PORT}"
HOST_DREAMVIEW_URL="http://${HOST_IP}:${HOST_DREAMVIEW_PORT}"

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

function start() {
  for mod in ${APOLLO_BOOTSTRAP_EXTRA_MODULES}; do
    info "Starting ${mod}"
    eval "nohup cyber_launch start ${mod} &" &>/dev/null
  done
  ./scripts/monitor.sh start
  ./scripts/dreamview.sh start
  if [ $? -eq 0 ]; then
    sleep 2 # wait for some time before starting to check
    http_status="$(curl -o /dev/null -x '' -I -L -s -w '%{http_code}' ${DREAMVIEW_URL})"
    if [ $http_status -eq 200 ]; then
      info "Dreamview is running at ${HOST_DREAMVIEW_URL}"
    else
      error "Failed to start Dreamview. Please check /apollo/data/log or /apollo/data/core for more information"
    fi
  fi
}

function stop() {
  ./scripts/dreamview.sh stop
  ./scripts/monitor.sh stop
  for mod in ${APOLLO_BOOTSTRAP_EXTRA_MODULES}; do
    echo "Stopping ${mod}"
    nohup cyber_launch stop ${mod}
  done
}

function main() {
  case "$1" in
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
}

main "$@"
