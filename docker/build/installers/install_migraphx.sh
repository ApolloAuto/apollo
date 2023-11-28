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

set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh


TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    PKG_NAME="migraphx.tar.gz"
    FILE_ID="1PiZP1YDU1scSY3kRz2qBLq3H35gteaHB"
else # AArch64
    error "AMD migraphx for ${TARGET_ARCH} not ready. Exiting..."
    exit 1
fi
DOWNLOAD_LINK="https://docs.google.com/uc?export=download&id=${FILE_ID}"
wget --load-cookies /tmp/cookies.txt \
    "https://docs.google.com/uc?export=download&confirm=
        $(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies \
        --no-check-certificate ${DOWNLOAD_LINK} \
        -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${FILE_ID}" \
    -O "${PKG_NAME}" && rm -rf /tmp/cookies.txt

tar xzf "${PKG_NAME}"
mv "${PKG_NAME%.tar.gz}" /opt/rocm/migraphx

cp -rLfs "/opt/rocm/migraphx/include/." "/opt/rocm/include/"
cp -rLfs "/opt/rocm/migraphx/lib/." "/opt/rocm/lib/"

# Cleanup
rm -f "${PKG_NAME}"
ok "Successfully installed migraphx"
