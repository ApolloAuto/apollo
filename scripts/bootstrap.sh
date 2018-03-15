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

function start() {
    # Setup supervisord.
    if [ "$HOSTNAME" == "in_release_docker" ]; then
        supervisord -c /apollo/modules/tools/supervisord/release.conf >& /tmp/supervisord.start.log
        echo "Started supervisord with release conf"
    else
        supervisord -c /apollo/modules/tools/supervisord/dev.conf >& /tmp/supervisord.start.log
        echo "Started supervisord with dev conf"
    fi

    # Start roscore.
    bash scripts/roscore.sh start
    # Start monitor.
    supervisorctl start monitor > /dev/null
    # Start dreamview.
    bash scripts/voice_detector.sh start
    supervisorctl start dreamview > /dev/null
    echo "Dreamview is running at http://localhost:8888"
}

function stop() {
    # Stop modules in reverse order of the starting procedure.
    supervisorctl stop dreamview
    bash scripts/voice_detector.sh stop
    supervisorctl stop monitor
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
