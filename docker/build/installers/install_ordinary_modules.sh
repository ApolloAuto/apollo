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

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

#######################################################

COMPONENT="modules/common"
info "Install support for [${COMPONENT}] ..."

info "Install osqp ..."
bash ${CURR_DIR}/install_osqp.sh

apt_get_update_and_install \
    libsqlite3-dev

######################################################

COMPONENT="modules/perception"
info "Install support for [${COMPONENT}] ..."
bash ${CURR_DIR}/install_paddle_deps.sh

######################################################

COMPONENT="modules/prediction"
info "Install support for [${COMPONENT}] ..."
bash ${CURR_DIR}/install_opencv.sh

#######################################################

COMPONENT="modules/planning"
info "Install support for [${COMPONENT}] ..."

bash ${CURR_DIR}/install_adolc.sh
bash ${CURR_DIR}/install_ipopt.sh

#######################################################

COMPONENT="modules/map"
info "Install support for [${COMPONENT}] ..."

apt_get_update_and_install \
    libtinyxml2-dev

# CUDA & nlohmann/json
#######################################################

COMPONENT="modules/localization"
info "Install support for [${COMPONENT}] ..."

ok "Good, no extra deps for localization. "

#######################################################
COMPONENT="modules/tools"
info "Install support for [${COMPONENT}] ..."
bash ${CURR_DIR}/install_python_modules.sh

# Modules that DON'T need pre-installed dependencies
# modules/v2x
# modules/storytelling
# modules/routing

######################################################
COMPONENT="modules/teleop"
info "Install support for [${COMPONENT}] ..."
bash ${CURR_DIR}/install_openh264.sh

######################################################
COMPONENT="modules/audio"
info "Install support for [${COMPONENT}] ..."
bash ${CURR_DIR}/install_fftw3.sh

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*

