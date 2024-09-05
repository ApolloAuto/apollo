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

# Fail on first error.
set -e

INSTALL_ATOM="${INSTALL_ATOM:-protobuf-3.14.0}"

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/cmake_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/cmake_package_install_funcs.sh

if ldconfig -p | grep -q "libprotobuf.so"; then
  info "Found existing Protobuf installation. Reinstallation skipped."
  exit 0
fi

# Notes on Protobuf Installer:
# 1) protobuf for cpp didn't need to be pre-installed into system
# 2) protobuf for python should be provided for cyber testcases

SRC_URI="${SRC_URI:-https://github.com/protocolbuffers/protobuf/archive/v${PV}.tar.gz -> ${PN}-${PV}.tar.gz}"

src_configure() {
  mkdir -p "${WORKDIR}/${PF}_build"
  pushd "${WORKDIR}/${PF}_build"
  if [[ -d "${WORKDIR}/${PF}/cmake" ]]; then
    cmake ${WORKDIR}/${PF}/cmake \
      -DBUILD_SHARED_LIBS=ON \
      -Dprotobuf_BUILD_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX:PATH="${INSTALL_PREFIX}" \
      -DCMAKE_BUILD_TYPE=Release
  else
    cmake ${WORKDIR}/${PF} \
      -DBUILD_SHARED_LIBS=ON \
      -Dprotobuf_BUILD_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX:PATH="${INSTALL_PREFIX}" \
      -DCMAKE_BUILD_TYPE=Release
  fi
  popd
}

src_install_post() {
  # Cf. https://github.com/protocolbuffers/protobuf/tree/master/python
  pushd "${WORKDIR}/${PF}/python"
  python3 setup.py install --cpp_implementation
  popd
  if grep -q "export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp" "${APOLLO_PROFILE}"; then
    :
  else
    echo -e "\nexport PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp" | tee -a "${APOLLO_PROFILE}"
  fi
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi
