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

# Prepare
PACKAGE="v2.0.0.tar.gz"
OPEN_H264="openh264-2.0.0"
wget https://github.com/cisco/openh264/archive/${PACKAGE}
tar zxf ${PACKAGE}

# Build and install.
pushd ${OPEN_H264}
  make
  make install
popd

# Clean
rm -fr ${PACKAGE} ${OPEN_H264}

