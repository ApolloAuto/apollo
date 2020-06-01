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

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

apt-get -y update && \
    apt-get -y install --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev

. /tmp/installers/installer_base.sh

PKG_NAME="fast-rtps-1.5.0.prebuilt.x86_64.tar.gz"
CHECKSUM="ca0534db4f757cb41a9feaebac07a13dd4b63af0a217b2cb456e20b0836bc797"
DOWNLOAD_LINK="https://apollo-platform-system.bj.bcebos.com/archive/6.0/${PKG_NAME}"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf ${PKG_NAME}
mv fast-rtps-1.5.0 /usr/local/fast-rtps

# TODO(storypku)
# As FastRTPS installer in other branches don't work well, we provided a prebuilt version
# here.
# Maybe the `cyber/transport/rtps` section needs a rewrite using a more recent FastRTPS impl.

rm -rf ${PKG_NAME}
