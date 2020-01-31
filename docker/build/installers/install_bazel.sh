#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

pushd "$(dirname "${BASH_SOURCE[0]}")"

# https://docs.bazel.build/versions/master/install-ubuntu.html
VERSION="2.1.0"
INSTALLER="bazel-${VERSION}-installer-linux-x86_64.sh"

apt update -y
apt install -y pkg-config zip g++ zlib1g-dev unzip python3
apt clean

wget "https://github.com/bazelbuild/bazel/releases/download/${VERSION}/${INSTALLER}"
bash "${INSTALLER}"
rm "${INSTALLER}"

popd
