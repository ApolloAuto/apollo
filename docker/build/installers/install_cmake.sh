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

#cmake-3.16.6-Linux-x86_64.sh
# Install CMake
VERSION=3.16.8
CMAKE_SH="cmake-${VERSION}-Linux-x86_64.sh"
SHA256SUM="0241a05bee0dcdf60e912057cc86cbedba21b9b0d67ec11bc67ad4834f182a23"

DOWLOAD_LINK=https://github.com/Kitware/CMake/releases/download/v${VERSION}/${CMAKE_SH}

download_if_not_cached $CMAKE_SH $SHA256SUM $DOWLOAD_LINK

chmod a+x ${CMAKE_SH}
mkdir -p /opt/cmake

./${CMAKE_SH} --skip-license --prefix=/opt/cmake
ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake

# Clean up.
rm -fr ${CMAKE_SH}
