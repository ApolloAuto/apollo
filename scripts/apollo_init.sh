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

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

# grant caros user to access GPS device
if [ -e /dev/ttyUSB0 ]; then
    sudo chown ${USER}:${USER} /dev/ttyUSB0 /dev/ttyUSB1
fi

# setup can device
if [ ! -e /dev/can0 ]; then
    sudo mknod --mode=a+rw /dev/can0 c 52 0
fi

if [ -e /dev/can0 ]; then
    sudo chown ${USER}:${USER} /dev/can0
fi

# enable coredump
echo "${APOLLO_ROOT_DIR}/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern >/dev/null 2>&1
