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

cd "$(dirname "${BASH_SOURCE[0]}")"

wget https://apollocache.blob.core.windows.net/apollo-cache/libjsonrpc-cpp.tar.gz
tar zxf libjsonrpc-cpp.tar.gz
mv libjsonrpc-cpp/bin/* /usr/local/bin/
mv libjsonrpc-cpp/include/* /usr/local/include/
mv libjsonrpc-cpp/lib/* /usr/local/lib/

# Clean up.
rm -fr libjsonrpc-cpp*
