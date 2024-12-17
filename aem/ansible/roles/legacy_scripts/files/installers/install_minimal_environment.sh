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
. ./installer_base.sh

MY_GEO=$1; shift
ARCH="$(uname -m)"

LSB_RELEASE=$1; shift

##----------------------------##
##  APT sources.list settings |
##----------------------------##

if [[ "${ARCH}" == "x86_64" ]]; then
    if [[ "${LSB_RELEASE}" == "20.04" ]]; then
        echo "currently apollo x86_64 image does not support ubuntu 22.04"
        exit -1
    fi
    if [[ "${MY_GEO}" == "cn" ]]; then
        cp -f "${RCFILES_DIR}/sources.list.cn.x86_64" /etc/apt/sources.list
        # sed -i 's/nvidia.com/nvidia.cn/g' /etc/apt/sources.list.d/nvidia-ml.list
    else
        sed -i 's/archive.ubuntu.com/us.archive.ubuntu.com/g' /etc/apt/sources.list
    fi
else # aarch64
    if [[ ! "${LSB_RELEASE}" == "20.04" ]]; then
        if [[ "${MY_GEO}" == "cn" ]]; then
            cp -f "${RCFILES_DIR}/sources.list.cn.aarch64" /etc/apt/sources.list
        fi
    fi
fi

# Disabled:
#   apt-file
apt_get_update_and_install \
    apt-utils \
    bc      \
    curl    \
    file    \
    gawk    \
    git     \
    less    \
    lsof    \
    python3     \
    python3-pip \
    python3-distutils \
    sed         \
    software-properties-common \
    sudo    \
    unzip   \
    vim     \
    wget    \
    zip     \
    xz-utils

if [[ "${ARCH}" == "aarch64" ]]; then
    apt-get -y install kmod
fi

MY_STAGE=
if [[ -f /etc/apollo.conf ]]; then
    MY_STAGE="$(awk -F '=' '/^stage=/ {print $2}' /etc/apollo.conf 2>/dev/null)"
fi

if [[ "${MY_STAGE}" != "runtime" ]]; then
    apt_get_update_and_install \
        build-essential \
        autoconf    \
        automake    \
        gcc-7       \
        g++-7       \
        gdb         \
        libtool     \
        patch       \
        pkg-config      \
        python3-dev     \
        libexpat1-dev   \
        linux-libc-dev
    # Note(storypku):
    # Set the last two packages to manually installed:
    #   libexpat1-dev was required by python3-dev
    #   linux-libc-dev was required by bazel/clang/cuda/...
fi

##----------------##
##    SUDO        ##
##----------------##
sed -i /etc/sudoers -re 's/^%sudo.*/%sudo ALL=(ALL:ALL) NOPASSWD: ALL/g'

##----------------##
## default shell  ##
##----------------##
chsh -s /bin/bash
ln -s /bin/bash /bin/sh -f

##----------------##
## Python Setings |
##----------------##
update-alternatives --install /usr/bin/python python /usr/bin/python3 36

if [[ "${MY_GEO}" == "cn" ]]; then
    # configure tsinghua's pypi mirror for x86_64 and aarch64
    PYPI_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple"
    pip3_install -i "$PYPI_MIRROR" pip -U
    python3 -m pip config set global.index-url "$PYPI_MIRROR"
else
    pip3_install pip -U
fi

pip3_install -U setuptools
pip3_install -U wheel

# Kick down the ladder
apt-get -y autoremove python3-pip

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
