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

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

source "${DIR}/apollo_base.sh"

function start() {
    DEBUG_MODE="yes"
    if [ "$HOSTNAME" == "in_release_docker" ]; then
        DEBUG_MODE="no"
    fi

    # Start roscore.
    bash scripts/roscore.sh start

    if [ "$DEBUG_MODE" == "yes" ]; then
        ./scripts/monitor.sh start
        ./scripts/dreamview.sh start
    else
        # Use supervisord.
        supervisord -c /apollo/modules/tools/supervisord/release.conf >& /tmp/supervisord.start.log
        echo "Started supervisord with release conf"

        # Start monitor.
        supervisorctl start monitor > /dev/null
        # Start dreamview.
        supervisorctl start dreamview
        supervisorctl status dreamview | grep RUNNING > /dev/null
    fi

    if [ $? -eq 0 ]; then
        echo "Dreamview is running at http://localhost:8888"
    fi
}

function stop() {
    DEBUG_MODE="yes"
    if [ "$HOSTNAME" == "in_release_docker" ]; then
        DEBUG_MODE="no"
    fi

    # Stop modules in reverse order of the starting procedure.
    if [ "$DEBUG_MODE" == "yes" ]; then
        ./scripts/dreamview.sh stop
        ./scripts/monitor.sh stop
    else
        supervisorctl stop dreamview
        supervisorctl stop monitor
    fi
    source scripts/roscore.sh stop
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
