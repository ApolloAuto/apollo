#!/usr/bin/env bash

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"

mkdir -p /opt/apollo/neo/src
dpkg -l apollo-neo-buildtool >/dev/null 2>&1
# install buildtool
[[ $? -ne 0 ]] && set -e && sudo rm -rf /etc/apt/keyrings/apolloauto.gpg && \
    mv /opt/apollo/neo/setup.sh /opt/apollo/neo/setup.sh.bak && \
    sudo apt-get install -y ca-certificates curl gnupg && sudo install -m 0755 -d /etc/apt/keyrings && \
    curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg && \
    sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg && echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core" \
    $(. /etc/os-release && echo "$VERSION_CODENAME") "main" | sudo tee /etc/apt/sources.list.d/apolloauto.list && \
    sudo apt-get update && sudo apt-get install -y apollo-neo-buildtool && \
    sudo touch /.installed && sudo sed -i 's/#include "flann\/general\.h"/#include <\/usr\/include\/flann\/general\.h>/g' /usr/include/flann/util/params.h

# install dv plugins
mkdir ~/apollo_tmp
cd ~/apollo_tmp
cp -r /apollo/modules ./
cp -r /apollo/cyber ./
if [ -e "/apollo/.workspace.json" ]; then
    cp /apollo/.workspace.json ./
fi
sudo rm -rf modules/studio_connector
sudo cp /etc/ld.so.conf.d/apollo.conf /etc/ld.so.conf.d/apollo_source.conf

buildtool init
buildtool install --legacy sim-obstacle studio-connector

# remove buildtool and tmp dir
sudo apt remove -y apollo-neo-buildtool
rm -rf ~/apollo_tmp
mv /opt/apollo/neo/setup.sh.bak /opt/apollo/neo/setup.sh
sudo ldconfig

ok "Successfully install dreamview plugins."
ok "Please restart dreamview. Enjoy!"
