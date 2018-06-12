#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

function check_docker_open() {
  docker ps --format "{{.Names}}" | grep apollo_dev 1>/dev/null 2>&1
  if [ $? != 0 ]; then       
    echo "The docker is not started, please start it first. "  
    exit 1  
  fi
}

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'

  echo -e "\n${RED}Usage${NONE}:
  .${BOLD}/dev_debug_server.sh${NONE} MODULE_NAME PORT_NUMBER"

  echo -e "${RED}MODULE_NAME${NONE}:
  ${BLUE}planning${NONE}: debug the planning module. 
  ${BLUE}control${NONE}: debug the control module.
  ${BLUE}routing${NONE}: debug the routing module.
  ..., and so on."

  echo -e "${RED}PORT_NUMBER${NONE}: 
  ${NONE}a port number, such as '1111'."
}

if [ $# -lt 2 ];then
    print_usage
    exit 1
fi

check_docker_open

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/../.."

xhost +local:root 1>/dev/null 2>&1
#echo $@
docker exec \
    -u $USER \
    -it apollo_dev \
    /bin/bash scripts/start_gdb_server.sh $@
xhost -local:root 1>/dev/null 2>&1
