#!/usr/bin/env bash

###############################################################################
# Copyright 2021 The Apollo Authors. All Rights Reserved.
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

INSTALL_ATOM="${INSTALL_ATOM:-openssl-1.1.1w}"

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/general_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/general_package_install_funcs.sh

if ldconfig -p | grep libssl | grep -q "${INSTALL_PREFIX}"; then
  info "OpenSSL was already installed"
  exit 0
fi

# Note: use for replacing boringssl in grpc

PV_NUM=$(echo "${PV}" | sed 's/[^0-9.]*//g')
#SRC_URI="${SRC_URI:-https://www.openssl.org/source/old/${PV_NUM}/${PN}-${PV}.tar.gz}"
SRC_URI="${SRC_URI:-https://apollo-system.cdn.bcebos.com/archive/10.0/${PN}-${PV}.tar.gz}"

src_configure() {
  pushd "${WORKDIR}/${PF}"
  ./config --prefix="${INSTALL_PREFIX}"
  popd
}

pkg_install_post() {
  if [[ -d "${INSTALL_PREFIX}/lib64" ]] && [[ ! -e "${INSTALL_PREFIX}/lib" ]]; then
    ln -snf lib64 "${INSTALL_PREFIX}/lib"
  fi
  ok "Done installing ${PN}-${PV}"
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi
