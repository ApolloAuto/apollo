#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. /tmp/installers/installer_base.sh

MY_GEO=$1; shift

##----------------------------##
##  APT sources.list settings |
##----------------------------##
echo "My geolocation is ${MY_GEO}"

PREV_GEO="us"
if grep -q "nvidia.cn" /etc/apt/sources.list.d/nvidia-ml.list ; then
    PREV_GEO="cn"
fi

if [[ "${MY_GEO}" == "${PREV_GEO}" ]]; then
    info "Skipped Geo Reconfiguration as ${MY_GEO}==${PREV_GEO}"
else
    warning "Perform Geo Adjustment from ${PREV_GEO} to ${MY_GEO}"
fi

# us->cn
if [ "$MY_GEO" == "cn" ]; then
    cp -f /tmp/installers/sources.list.cn /etc/apt/sources.list
    sed -i 's/nvidia.com/nvidia.cn/g' /etc/apt/sources.list.d/nvidia-ml.list
    # Mirror from Tsinghua Univ.
    PYPI_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple"
    pip config set global.index-url "$PYPI_MIRROR"
    python3 -m pip config set global.index-url "$PYPI_MIRROR"
elif [ "$MY_GEO" == "us" ]; then
    cp -f /tmp/installers/sources.list.us /etc/apt/sources.list
    sed -i 's/nvidia.cn/nvidia.com/g' /etc/apt/sources.list.d/nvidia-ml.list
    PYPI_MIRROR="https://pypi.org/simple"
    pip config set global.index-url "$PYPI_MIRROR"
    python3 -m pip config set global.index-url "$PYPI_MIRROR"
fi

