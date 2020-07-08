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
. /tmp/installers/installer_base.sh

curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add -

# Ref https://classic.yarnpkg.com/en/docs/install/#debian-stable
# Don't use tee here. Or else it complains
# "Warning: apt-key output should not be parsed (stdout is not a terminal)"
echo "deb https://dl.yarnpkg.com/debian/ stable main" > /etc/apt/sources.list.d/yarn.list

apt-get -y update && \
    apt-get -y --no-install-recommends install \
    yarn

info "Successfully installed yarn"
apt-get clean
rm -fr /etc/apt/sources.list.d/yarn.list
