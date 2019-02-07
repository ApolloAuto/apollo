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

# download the pre-built fast-rtps
# from the apollo-platform source code
wget https://github.com/ApolloAuto/apollo-platform/archive/2.1.2.tar.gz

# install pre-built for x86_64
tar -xf 2.1.2.tar.gz
cp -r apollo-platform-2.1.2/ros/third_party/fast-rtps_x86_64 /usr/local/fast-rtps

#clean up
rm -rf 2.1.2.tar.gz apollo-platform-2.1.2

