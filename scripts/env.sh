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

if [ -f /.dockerenv ]; then
    APOLLO_IN_DOCKER=true
else
    APOLLO_IN_DOCKER=false
fi

hostname
if $APOLLO_IN_DOCKER; then
  set -x
  echo "Inside docker"
  uname -a
  pip list
else
  echo "Outside docker"
  set -x
  uname -a
  docker --version
  docker images | grep apollo
fi
echo "-----------env---------------"
env
