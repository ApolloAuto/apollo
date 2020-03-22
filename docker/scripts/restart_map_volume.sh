#!/usr/bin/env bash

# Usage:
#    restart_map_volume.sh <map_name> <map_version>

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

map_name=$1
map_version=$2

MAP_VOLUME="apollo_map_volume-${map_name}_${USER}"
if [[ ${MAP_VOLUME_CONF} == *"${MAP_VOLUME}"* ]]; then
  echo "Map ${map_name} has already been included!"
else
  docker stop ${MAP_VOLUME} > /dev/null 2>&1

  MAP_VOLUME_IMAGE=${DOCKER_REPO}:map_volume-${map_name}-${map_version}
  do_docker_pull ${MAP_VOLUME_IMAGE}
  docker run -it -d --rm --name ${MAP_VOLUME} ${MAP_VOLUME_IMAGE}
  MAP_VOLUME_CONF="${MAP_VOLUME_CONF} --volumes-from ${MAP_VOLUME}"
fi
