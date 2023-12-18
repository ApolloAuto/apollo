#!/usr/bin/env bash

###############################################################################
# Copyright 2017-2021 The Apollo Authors. All Rights Reserved.
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
DREAMVIEW_URL="http://localhost:8899"
DREAMVIEW_PLUS_URL="http://localhost:8888"

cd "${DIR}/.."

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

source "${DIR}/apollo_base.sh"

function start() {
  for mod in ${APOLLO_BOOTSTRAP_EXTRA_MODULES}; do
    echo "Starting ${mod}"
    nohup cyber_launch start ${mod} &
  done
  ./scripts/monitor.sh start
  ./scripts/dreamview.sh start
  if [ $? -eq 0 ]; then
    sleep 2 # wait for some time before starting to check
    http_status="$(curl -o /dev/null -x '' -I -L -s -w '%{http_code}' ${DREAMVIEW_URL})"
    if [ $http_status -eq 200 ]; then
      echo "Dreamview is running at" $DREAMVIEW_URL
    else
      echo "Failed to start Dreamview. Please check /apollo/nohup.out or /apollo/data/core for more information"
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


function start_plus() {
  for mod in ${APOLLO_BOOTSTRAP_EXTRA_MODULES}; do
    echo "Starting ${mod}"
    nohup cyber_launch start ${mod} &
  done
  ./scripts/monitor.sh start
  ./scripts/dreamview_plus.sh start
  if [ $? -eq 0 ]; then
    sleep 2 # wait for some time before starting to check
    http_status="$(curl -o /dev/null -x '' -I -L -s -w '%{http_code}' ${DREAMVIEW_PLUS_URL})"
    if [ $http_status -eq 200 ]; then
      echo "Dreamview Plus is running at" $DREAMVIEW_PLUS_URL
    else
      echo "Failed to start Dreamview Plus. Please check /apollo/nohup.out or /apollo/data/core for more information"
    fi
  fi
}

function stop_plus() {
  ./scripts/dreamview_plus.sh stop
  ./scripts/monitor.sh stop
  for mod in ${APOLLO_BOOTSTRAP_EXTRA_MODULES}; do
    echo "Stopping ${mod}"
    nohup cyber_launch stop ${mod}
  done
}

case $1 in
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
  start_plus)
    start_plus
    ;;
  stop_plus)
    stop_plus
    ;;
  restart_plus)
    stop_plus
    start_plus
    ;;
  *)
    start
    ;;
esac
