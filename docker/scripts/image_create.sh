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

if [ $# != 1 ];then
  echo 'Usage:
  ./image_create.sh [OPTION]'
  echo 'Options:
  dev: create dev image
  run: create release base image
  '
  exit 1
fi

TYPE=$1
echo $TYPE
TIME=$(date  +%Y%m%d_%H%M)
if [ -z "${DOCKER_REPO}" ]; then
    DOCKER_REPO=apolloauto/apollo
fi


APOLLO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
ARCH=$(uname -m)
TAG="${TYPE}-${ARCH}-${TIME}"

# Build image from APOLLO_ROOT, while use the specified Dockerfile.
docker build -t "${DOCKER_REPO}:tmp_tag" \
    -f "${APOLLO_ROOT}/docker/${TYPE}.${ARCH}.dockerfile" \
    "${APOLLO_ROOT}"

if [ $? != 0 ]; then
  echo "Building image failed!"
  exit 1
fi

# Flatten the image to one layer.
docker run "${DOCKER_REPO}:tmp_tag"
if [ $? != 0 ]; then
  echo "Starting container failed!"
  exit 1
fi

CONTAINER_ID=$(docker ps -a --format "{{.ID}}" | awk NR==1)
docker export $CONTAINER_ID | docker import - ${DOCKER_REPO}:${TAG}
docker rmi -f "${DOCKER_REPO}:tmp_tag"
