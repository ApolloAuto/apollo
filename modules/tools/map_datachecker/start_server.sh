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

set -ue
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APOLLO_ROOT_PATH="${SCRIPT_PATH}/../../.."
MAP_DATACHECKER=${APOLLO_ROOT_PATH}/bazel-bin/modules/map/tools/map_datachecker/map_datachecker
CONF=${APOLLO_ROOT_PATH}/modules/map/tools/map_datachecker/conf/map-datachecker.conf
${MAP_DATACHECKER} --flagfile=${CONF} > ${SCRIPT_PATH}/`date '+%Y-%m-%d--%H-%M-%S'`.log 2>&1 &
echo 'Server has been started successfully'
echo 'You can enter Ctrl+C to exit this server'
PID=$!
trap "trap_func" INT
function trap_func() {
  echo 'Stopping map_datachecker......'
  kill -INT ${PID}
}
while [[ true ]]; do
  sleep 2
done












