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

set -ex

action="install"

if [[ $# -ge 1 ]]; then
  action="$1"
fi

DEST_DIR_BASE="third_party/can_card_library/esd_can"
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DEST_DIR="${CURR_DIR}/../${DEST_DIR_BASE}"

[[ -d ${DEST_DIR}/include ]] && rm -rf ${DEST_DIR}/include
[[ -d ${DEST_DIR}/lib ]] && rm -rf ${DEST_DIR}/lib

if [[ "${action}" == "install" ]]; then
  mkdir -p "${DEST_DIR}/include"
  mkdir -p "${DEST_DIR}/lib"
  CONTRIB_REPO="${CURR_DIR}/../../apollo-contrib"
  if [[ -d "${CONTRIB_REPO}" ]]; then
    echo "apollo-contrib found."
  else
    sudo git clone https://github.com/ApolloAuto/apollo-contrib.git ${CONTRIB_REPO}
  fi
  pushd "${CONTRIB_REPO}"
  cp esd/include/* ${DEST_DIR}/include/
  cp esd/lib64/libntcan.so.4.0.1 ${DEST_DIR}/lib
  popd

  pushd ${DEST_DIR}/lib
  ln -s libntcan.so.4.0.1 libntcan.so.4.0
  ln -s libntcan.so.4.0.1 libntcan.so.4
  popd
fi
