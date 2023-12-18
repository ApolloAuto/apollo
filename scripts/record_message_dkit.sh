#!/usr/bin/env bash

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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

# """
# Save the full data into /apollo/data/bag/ReusedRecordsPool,
# which will be cleared every time the smart recorder get started.
# Meanwhile, restore the messages we are interested in to /apollo/data/bag/<task_id> directory.
# """
function start() {
  local APOLLO_ROOT_DIR="/apollo"
  local APOLLO_BIN_PREFIX="${APOLLO_ROOT_DIR}/bazel-bin"
  TIME="$(date +%F-%H-%M-%S)"
  MODULE="smart_recorder"

  REUSE_POOL_DIR="${APOLLO_ROOT_DIR}/data/ReusedRecordsPool"
  TASK_EXE_DIR="${APOLLO_ROOT_DIR}/data/${MODULE}/${TIME}"
  LOG_DIR="${APOLLO_ROOT_DIR}/data/log"
  mkdir -p $LOG_DIR
  sudo chmod -R 777 $LOG_DIR 
  LOG="${LOG_DIR}/smart_recorder.out"
  RECORD_EXE="${APOLLO_BIN_PREFIX}/modules/data/tools/${MODULE}/${MODULE}"
  if [[ ! -f ${RECORD_EXE} ]]; then
    RECORD_EXE="/opt/apollo/neo/packages/apollo-data-dkit-dev/latest/bin/tools/${MODULE}/${MODULE}"
  fi

  if [[ ! -f ${RECORD_EXE} ]]; then
    echo "can't find smart_recorder. Have you installed apollo-data-dkit-dev?"
    exit -1
  fi

  NUM_PROCESSES="$(pgrep -f "smart_recorder" | grep -cv '^1$')"
  if [ "${NUM_PROCESSES}" -ne 0 ]; then
    pkill -SIGINT -f smart_recorder
  fi

  nohup ${RECORD_EXE} --source_records_dir=${REUSE_POOL_DIR} \
    --restored_output_dir=${TASK_EXE_DIR} < /dev/null > ${LOG} 2>&1 &
}

function stop() {
  pkill -SIGINT -f smart_recorder
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
  *)
    start
    ;;
esac
