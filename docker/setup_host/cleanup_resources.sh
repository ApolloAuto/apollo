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

if [ ! -z ${DOCKER_IMG} ]; then
  echo "This script is expected to be run on host instead of the container."
  echo "Please exit."
  exit 1
fi

# Credit to https://gist.github.com/bastman/5b57ddb3c11942094f8d0a97d461b430
echo "Cleanup containers..."
docker rm $(docker ps -qa --no-trunc --filter "status=exited")

echo "Cleanup images..."
docker rmi $(docker images --filter "dangling=true" -q --no-trunc)

echo "Cleanup volumes..."
docker volume rm $(docker volume ls -qf dangling=true)
