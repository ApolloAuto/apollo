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

# Ubuntu 14.04 has poco package version 1.3.6
# if a higher version is required, the below
# will install from source
apt-get -y update && \
    apt-get -y install poco

# Install from source
#VERSION=1.9.0

#wget https://github.com/pocoproject/poco/archive/poco-${VERSION}-release.tar.gz
#tar -xf poco-${VERSION}-release.tar.gz

# we cant use cmake because poco requires > 3.2
# and the container is at 2.8
# but standard ./configure && make works fine

#pushd poco-poco-${VERSION}-release
  #./configure --omit=Data/ODBC,Data/MySQL && \
      #    make -s -j`nproc` && \
      #    make -s -j`nproc` install
#popd

# clean up
#rm -rf poco-${VERSION}-release.tar.gz poco-poco-${VERSION}-release
