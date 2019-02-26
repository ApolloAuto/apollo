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
DREAMVIEW_URL="http://localhost:8888"

cd "${DIR}/.."

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

source "${DIR}/apollo_base.sh"

function start() {
    ./scripts/monitor.sh start
    ./scripts/dreamview.sh start
    if [ $? -eq 0 ]; then
        sleep 2  # wait for some time before starting to check
        http_status="$(curl -o /dev/null -I -L -s -w '%{http_code}' ${DREAMVIEW_URL})"
        if [ $http_status -eq 200 ]; then
            echo "Dreamview is running at" $DREAMVIEW_URL
        else
            echo "Failed to start Dreamview. Please check /apollo/data/log or /apollo/data/core for more information"
        fi
    fi
}

function stop() {
    ./scripts/dreamview.sh stop
    ./scripts/monitor.sh stop
}

case $1 in
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
