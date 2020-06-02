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

INSTALL_MODE=$1; shift;

cd "$(dirname "${BASH_SOURCE[0]}")"
. /tmp/installers/installer_base.sh

#######################################################

COMPONENT="modules/common"
info "Install support for [${COMPONENT}] ..."

info "Install osqp ..."
bash /tmp/installers/install_osqp.sh

info "Install qpOASES ..."
bash /tmp/installers/install_qp_oases.sh


#######################################################

COMPONENT="modules/transform"
info "Install support for [${COMPONENT}] ..."
bash /tmp/installers/install_tf2.sh

######################################################

COMPONENT="modules/prediction"
info "Install support for [${COMPONENT}] ..."
# bash /tmp/installers/install_libtorch.sh
apt-get -y update && \
    apt-get -y install \
    libopencv-core-dev \
    libopencv-imgproc-dev \
    libopencv-imgcodecs-dev \
    libopencv-highgui-dev \
    libboost-all-dev

#######################################################

COMPONENT="modules/planning"
info "Install support for [${COMPONENT}] ..."

# bash /tmp/installers/install_libtorch.sh
bash /tmp/installers/install_adolc.sh
bash /tmp/installers/install_ipopt.sh
# [TO-BE-CONTINUED]

#######################################################

COMPONENT="modules/map"
info "Install support for [${COMPONENT}] ..."

apt-get -y update && \
    apt-get -y install \
    libtinyxml2-dev \
    libboost-all-dev
# CUDA & nlohmann/json

#######################################################

COMPONENT="modules/monitor"
info "Install support for [${COMPONENT}] ..."

if dpkg -l | grep -q "linux-libc-dev"; then
    info "linux-libc-dev already installed"
else
    apt-get -y update && \
        ap-get -y install \
        linux-libc-dev
fi

#######################################################
COMPONENT="modules/localization"
info "Install support for [${COMPONENT}] ..."

apt-get -y update && \
    apt-get -y install \
    liblz4-dev

# Modules that DON'T need pre-installed dependencies
# modules/v2x
# modules/third_party_perception
# modules/storytelling
# modules/routing

