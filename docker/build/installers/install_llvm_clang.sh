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

# Install clang via apt to reduce image size
apt_get_update_and_install \
    clang-10 \
    clang-format-10

#    clang-tidy-10 \
#    clang-tools-10

sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-10 100
sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-10 100

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install from source
# Ref:
# https://releases.llvm.org/download.html
# https://github.com/llvm/llvm-project/releases

ok "Done installing LLVM Clang-10."
