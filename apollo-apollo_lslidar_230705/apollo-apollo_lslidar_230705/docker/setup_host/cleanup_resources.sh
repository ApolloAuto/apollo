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

if [ -f /.dockerenv ]; then
  echo "Oops, this script is expected to run on host rather than from within container."
  exit 1
fi

# Credit to https://gist.github.com/bastman/5b57ddb3c11942094f8d0a97d461b430
exited_containers="$(docker ps -qa --no-trunc --filter "status=exited")"
if [ -z "${exited_containers}" ]; then
    echo "Congrats, no exited docker containers found."
else
    echo "Exited containers found, cleanup ..."
    docker rm ${exited_containers}
fi

dangling_images="$(docker images --filter "dangling=true" -q --no-trunc)"
if [ -z "${dangling_images}" ]; then
    echo "Congrats, no dangling docker images found."
else
    echo "Dangling images found, cleanup ..."
    docker rmi ${dangling_images}
fi

dangling_volumes="$(docker volume ls -qf dangling=true)"
if [ -z "${dangling_volumes}" ]; then
    echo "Congrats, no dangling docker volumes found."
else
    echo "Dangling volumes found, cleanup ..."
    docker volume rm ${dangling_volumes}
fi

echo "Docker cleaning up finished."
