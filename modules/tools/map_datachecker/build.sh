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

set -xeu
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APOLLO_ROOT_PATH="${SCRIPT_PATH}/../../.."
# MAP_DATACHECKER_PROTO_PATH="${APOLLO_ROOT_PATH}/modules/map/tools/map_datachecker/proto"
MAP_DATACHECKER_PROTO_PATH="${APOLLO_ROOT_PATH}/modules/tools/map_datachecker/proto"

WORK_PATH=$(pwd)
# export LD_LIBRARY_PATH=${WORK_PATH}/../../../third_party/protobuf/lib:$LD_LIBRARY_PATH
export PYTHONPATH=${SCRIPT_PATH}:${PYTHONPATH}
# export PYTHONPATH=/home/yyj/.cache/bazel/_bazel_yyj/fc32bb8f580b2c0edf12f203675eb3f2/external/com_github_grpc_grpc/tools/distrib/python/grpcio_tools:${PYTHONPATH}

PROTOC="protoc"
PROTO_OUT="${SCRIPT_PATH}/py_proto"

if [ -d ${PROTO_OUT} ];then
  rm -rf ${PROTO_OUT}
fi
mkdir -p ${PROTO_OUT}
files=`echo $(find ${MAP_DATACHECKER_PROTO_PATH} -name *.proto)`
dirs=`echo $(find ${MAP_DATACHECKER_PROTO_PATH}/* -type d | awk '{print "-I"$0}')`
${PROTOC} ${dirs} --python_out=${PROTO_OUT} --proto_path=${MAP_DATACHECKER_PROTO_PATH} ${files}
[ $? -eq 0 ] &&
    { echo "generate *.py from *.proto successfully."; touch "${PROTO_OUT}/__init__.py"; } ||
    { echo "generate py from proto failed."; exit 1; }

services=`echo $(grep "^service" ${files} | awk -F: '{print $1}' | uniq)`
python -m grpc_tools.protoc ${dirs} --python_out=${PROTO_OUT} --grpc_python_out=${PROTO_OUT} ${services} --proto_path=${MAP_DATACHECKER_PROTO_PATH}

[ $? -eq 0 ] &&
    { echo "generate grpc *.py from *.proto successfully."; touch "${PROTO_OUT}/__init__.py"; } ||
    { echo "generate grpc py from proto failed."; exit 1; }


