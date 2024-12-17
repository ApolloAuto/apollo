#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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
set -e

INSTALL_MODE=${INSTALL_MODE:-binary}
INSTALL_ATOM="${INSTALL_ATOM:-fast-rtps-1.5.0}"

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/cmake_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/cmake_package_install_funcs.sh

if ldconfig -p | grep -q libfastrtps; then
  info "Fast-RTPS was already installed"
  exit 0
fi

INSTALL_PREFIX="/usr/local/fast-rtps"

TARGET_ARCH="$(uname -m)"
if [[ "${INSTALL_MODE}" == "binary" ]]; then
  if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    SRC_URI="https://apollo-system.cdn.bcebos.com/archive/6.0/fast-rtps-1.5.0-1.prebuilt.x86_64.tar.gz"
  elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    SRC_URI="https://apollo-system.cdn.bcebos.com/archive/6.0/fast-rtps-1.5.0-1.prebuilt.aarch64.tar.gz"
  else
    error "Unsupported target arch: ${TARGET_ARCH}"
    exit 1
  fi
else
  # build from source
  # SRC_URI="https://github.com/eProsima/Fast-DDS/archive/refs/tags/v1.5.0.tar.gz -> ${PN}-${PV}.tar.gz"
  GIT_REPO_URI="https://github.com/eProsima/Fast-DDS.git"
  GIT_BRANCH="v1.5.0"
  GIT_RECURSIVE="true"
  GIT_DEPTH=1
  PATCHES=(
    "${FILESDIR}/FastRTPS_1.5.0.patch"
  )
fi

src_prepare_pre() {
  apt_get_update_and_install \
    libasio-dev \
    libtinyxml2-dev
}

# Note(storypku)
# 1) More recent Fast-DDS (formerly Fast-RTPS) implementations:
# Ref: https://github.com/eProsima/Fast-DDS
# Ref: https://github.com/ros2/rmw_fastrtps
# 2) How to create the diff
# git diff --submodule=diff > /tmp/FastRTPS_1.5.0.patch
# Ref: https://stackoverflow.com/questions/10757091/git-list-of-all-changed-files-including-those-in-submodules

# src_prepare() {
#   cmake_src_prepare
#   if [[ "${INSTALL_MODE}" == "source" ]]; then
#     pushd ${WORKDIR}/${PF}
#     git submodule update --init --recursive
#     popd
#   fi
# }

src_configure() {
  if [[ "${INSTALL_MODE}" == "binary" ]]; then
    return
  fi
  cmake_src_configure
}

src_compile() {
  if [[ "${INSTALL_MODE}" == "binary" ]]; then
    return
  fi
  cmake_src_compile
}

src_install() {
  if [[ "${INSTALL_MODE}" == "binary" ]]; then
    mkdir -p ${INSTALL_PREFIX}
    cp -arpv ${WORKDIR}/${PF}/* ${INSTALL_PREFIX}/
    return
  fi
  cmake_src_install
}

pkg_install_post() {
  :
  # apt_get_remove libasio-dev libtinyxml2-dev
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi
