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
source "${DIR}/scripts/apollo.bashrc"

DEV_CONTAINER="apollo_dev_${USER}"

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'

  echo -e "\n${RED}Usage${NONE}:
  ${BOLD}./apollo_docker.sh${NONE} [OPTION]"

  echo -e "\n${RED}Options${NONE}:
  ${BLUE}build${NONE}: run build only
  ${BLUE}build_opt${NONE}: build optimized binary for the code
  ${BLUE}build_gpu${NONE}: run build only with Caffe GPU mode support
  ${BLUE}build_opt_gpu${NONE}: build optimized binary with Caffe GPU mode support
  ${BLUE}build_fe${NONE}: compile frontend JS code (requires all required node modules already installed)
  ${BLUE}buildify${NONE}: fix style of BUILD files
  ${BLUE}check${NONE}: run build/lint/test, please make sure it passes before checking in new code
  ${BLUE}clean${NONE}: run Bazel clean
  ${BLUE}config${NONE}: run configurator tool
  ${BLUE}coverage${NONE}: generate test coverage report
  ${BLUE}doc${NONE}: generate doxygen document
  ${BLUE}lint${NONE}: run code style check
  ${BLUE}test${NONE}: run all unit tests
  ${BLUE}usage${NONE}: print this menu
  ${BLUE}version${NONE}: display current commit and date
  "
}

function start_docker() {
  if docker ps --format "{{.Names}}" | grep -q "${DEV_CONTAINER}" ; then
    return
  fi
  # If Google is reachable, we fetch the docker image directly.
  if ping -q -c 1 -W 1 www.google.com &>/dev/null ; then
    opt=""
  else
    if ! ping -q -c 1 -W 1 www.baidu.com &>/dev/null; then
    # If Baidu is unreachable, we use local images.
      opt="-l"
    fi
  fi
  bash docker/scripts/dev_start.sh ${opt}
}

# RELEASE_NAME=$(docker images --format "{{.Repository}}:{{.Tag}} {{.ID}}"
function main() {
  if [ $# -eq 0 ];then
    print_usage
    exit 1
  fi
  option="$1"
  case $option in
  -h|--help)
    print_usage
    exit 1
    ;;
  *)
    start_docker
    docker exec -u "${USER}" "${DEV_CONTAINER}" bash -c "./apollo.sh $*"
    ;;
  esac
}


main "$@"
