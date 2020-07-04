#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
# Prepare
apt-get update -y && \
    apt-get install -y \
        libcrypto++-dev \
        libcurl4-openssl-dev \
        libfuse-dev \
        libssl-dev \
        uuid-dev

VERSION=1.0.0
PACKAGE="bosfs-${VERSION}.10.tar.gz"
DOWNLOAD_LINK="http://sdk.bce.baidu.com/console-sdk/${PACKAGE}"
CHECKSUM="83999e2a8ec7a9ebb1afe462ac898ec95d887391c94375d94d607fba35b9133b"

download_if_not_cached "${PACKAGE}" "$CHECKSUM" "$DOWNLOAD_LINK"

tar zxf ${PACKAGE}

# Build and install.
pushd bosfs-${VERSION}
    sed -i '/cd bosfs/d' build.sh
    bash build.sh
popd

ok "Successfully installed bosfs-${VERSION}."

# Clean
rm -fr ${PACKAGE} "bosfs-${VERSION}"
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
