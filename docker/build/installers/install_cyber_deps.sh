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

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

TARGET_ARCH="$(uname -m)"

apt-get -y update && \
    apt-get -y install \
    ncurses-dev \
    libuuid1 \
    uuid-dev

info "Install protobuf ..."
bash ${CURR_DIR}/install_protobuf.sh

info "Install fast-rtps ..."
bash ${CURR_DIR}/install_fast-rtps.sh

# absl
bash ${CURR_DIR}/install_abseil.sh

# gflags and glog
bash ${CURR_DIR}/install_gflags_glog.sh

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# libc patch
wget "https://apollo-system.cdn.bcebos.com/archive/9.0/libc6-2.31-ubuntu18-amd64.tar.gz" \
    -O libc6-2.31-ubuntu18-amd64.tar.gz
tar -xzvf libc6-2.31-ubuntu18-amd64.tar.gz
apt-get -y install ./*.deb
rm -f libc*