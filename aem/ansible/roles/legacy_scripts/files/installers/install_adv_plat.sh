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

# MY_MODE="${1:-download}"
INSTALL_MODE=${INSTALL_MODE:-source}
INSTALL_ATOM=${INSTALL_ATOM:-adv_plat-3.0.1}

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/general_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/general_package_install_funcs.sh

if ldconfig -p | grep -q libadv_trigger; then
  info "adv_plat was already installed"
  exit 0
fi

TARGET_ARCH="$(uname -m)"

DEST_DIR="${PKGS_DIR}/adv_plat"
[[ -d ${DEST_DIR} ]] && rm -rf ${DEST_DIR}
mkdir -p ${DEST_DIR}

if [[ "${INSTALL_MODE}" == "source" ]]; then
  SRC_URI="https://apollo-system.cdn.bcebos.com/archive/6.0/apollo-contrib-baidu-1.0.tar.gz"
elif [[ "${INSTALL_MODE}" == "binary" ]]; then
  SRC_URI="https://apollo-system.cdn.bcebos.com/archive/6.0/${PN}-${PV}-${TARGET_ARCH}.tar.gz"
fi

src_configure() {
  :
}

src_compile() {
  if [[ "${INSTALL_MODE}" == "binary" ]]; then
    return
  elif [[ "${INSTALL_MODE}" == "source" ]]; then
    pushd "${WORKDIR}/${PF}/baidu/src/lib/adv_trigger"
    make -j$(nproc)
    make install
    make clean
    popd
    pushd "${WORKDIR}/${PF}/baidu/src/lib/bcan"
    make -j$(nproc)
    make install
    make clean
    popd
  fi
}

src_install() {
  if [[ "${INSTALL_MODE}" == "binary" ]]; then
    mkdir -p ${INSTALL_PREFIX}
    cp -arpv "${WORKDIR}/${PN}-${PV}"/* ${INSTALL_PREFIX}/
    return
  elif [[ "${INSTALL_MODE}" == "source" ]]; then
    pushd "${WORKDIR}/${PF}/baidu/output"
    cp -r ../src/kernel/include/uapi/linux include/
    rm -rf lib/libadv_*.a
    create_so_symlink lib
    mkdir -p ${INSTALL_PREFIX}
    cp -arpv include ${INSTALL_PREFIX}/
    cp -arpv lib ${INSTALL_PREFIX}/
    popd
  fi
}

# if [[ "${MY_MODE}" == "download" ]]; then
#   if [[ "${ARCH}" == "x86_64" ]]; then
#     PKG_NAME="adv_plat-3.0.1-x86_64.tar.gz"
#     CHECKSUM="3853ff381f8cb6e1681c73e7b4fbc004e950b83db890e3fa0bab9bf6685114ac"
#     DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
#     download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
#
#     tar xzf ${PKG_NAME}
#
#     # Note(storypku): workaround for the issue that adv_plat built on ubuntu 20.04 host
#     # can't work properly for camera driver
#     EXTRACTED_PKG="${PKG_NAME%%.tar.gz}"
#     mv -f ${EXTRACTED_PKG}/* ${DEST_DIR}/
#     echo "${DEST_DIR}/lib" >> "${APOLLO_LD_FILE}"
#     ldconfig
#
#     # cleanup
#     rm -rf ${EXTRACTED_PKG} ${PKG_NAME}
#     exit 0
#   fi
# fi
#
# #info "Git clone https://github.com/ApolloAuto/apollo-contrib.git"
# #git clone https://github.com/ApolloAuto/apollo-contrib.git
#
# PKG_NAME="apollo-contrib-baidu-1.0.tar.gz"
# CHECKSUM="cd385dae6d23c6fd70c2c0dcd0ce306241f84a638f50988c6ca52952c304bbec"
# DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
#
# download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
#
# tar xzf ${PKG_NAME}
#
# BAIDU_DIR="apollo-contrib/baidu"
# SRC_DIR="${BAIDU_DIR}/src/lib"
# OUT_DIR="${BAIDU_DIR}/output"
#
# pushd ${SRC_DIR}
# pushd adv_trigger
# make -j$(nproc)
# make install
# make clean
# popd
# pushd bcan
# make -j$(nproc)
# make install
# make clean
# popd
# popd
#
# LINUX_HEADERS="../src/kernel/include/uapi/linux"
#
# pushd ${OUT_DIR}
# cp -r ${LINUX_HEADERS} include/
# rm -rf lib/libadv_*.a
#
# create_so_symlink lib
#
# mkdir -p "${DEST_DIR}"
# mv include ${DEST_DIR}
# mv lib ${DEST_DIR}
#
# echo "${DEST_DIR}/lib" >> "${APOLLO_LD_FILE}"
# ldconfig
# popd
#
# rm -rf ${PKG_NAME} apollo-contrib

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi
