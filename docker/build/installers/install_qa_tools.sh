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

## buildifier ##
PKG_NAME="buildifier"
CHECKSUM="e92a6793c7134c5431c58fbc34700664f101e5c9b1c1fcd93b97978e8b7f88db"
DOWNLOAD_LINK="https://github.com/bazelbuild/buildtools/releases/download/3.0.0/buildifier"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

chmod a+x ${PKG_NAME}
cp ${PKG_NAME} /usr/local/bin/
rm -rf ${PKG_NAME}

apt-get -y update && \
    apt-get -y install \
    cppcheck \
    shellcheck \
    lcov

## Pylint
python3 -m pip install pylint

apt-get clean && \
    rm -rf /var/lib/apt/lists/*
