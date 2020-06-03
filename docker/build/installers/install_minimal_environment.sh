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

MY_GEO=$1; shift
ARCH="$(uname -m)"

##----------------------------##
##  APT sources.list settings |
##----------------------------##
. /tmp/installers/installer_base.sh

if [[ "${ARCH}" == "x86_64" ]]; then
    if [[ "${MY_GEO}" == "cn" ]]; then
        cp -f "${RCFILES_DIR}/sources.list.cn.x86_64" /etc/apt/sources.list
        # sed -i 's/nvidia.com/nvidia.cn/g' /etc/apt/sources.list.d/nvidia-ml.list
    else
        sed -i 's/archive.ubuntu.com/us.archive.ubuntu.com/g' /etc/apt/sources.list
    fi
else # aarch64
    if [[ "${MY_GEO}" == "cn" ]]; then
        cp -f "${RCFILES_DIR}/sources.list.cn.aarch64" /etc/apt/sources.list
    fi
fi

apt-get -y update && \
    apt-get install -y --no-install-recommends \
    apt-utils

apt-get -y update && \
    apt-get -y install -y --no-install-recommends \
    build-essential \
    autotools-dev \
    apt-file \
    sudo \
    gcc-7 \
    g++-7 \
    gdb \
    wget \
    curl \
    git \
    vim \
    sed \
    gawk \
    bc \
    patch \
    tree \
    lsof \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    openssh-client \
    software-properties-common \
    unzip \
    zip

##----------------##
##    SUDO        ##
##----------------##
sed -i /etc/sudoers -re 's/^%sudo.*/%sudo ALL=(ALL:ALL) NOPASSWD: ALL/g'

##
##----------------##
## Python Setings |
##----------------##

if [[ "${MY_GEO}" == "cn" ]]; then
    if [[ "${ARCH}" == "x86_64" ]]; then
        # Mirror from Tsinghua Univ.
        PYPI_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple"
        python3 -m pip install --no-cache-dir -i "$PYPI_MIRROR" pip -U
        python3 -m pip config set global.index-url "$PYPI_MIRROR"
    else
        info "Use default PYPI mirror: https://pypi.org/simple for ${ARCH} for ${MY_GEO}"
        python3 -m pip install --no-cache-dir pip -U
    fi
else
    python3 -m pip install --no-cache-dir pip -U
fi

python3 -m pip install --no-cache-dir setuptools

# Clean up.
apt-get clean && rm -rf /var/lib/apt/lists/*

