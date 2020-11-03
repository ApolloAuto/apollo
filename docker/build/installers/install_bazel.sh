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

TARGET_ARCH=$(uname -m)

BAZEL_VERSION="3.5.0"
BUILDTOOLS_VERSION="3.5.0"

if [[ "$TARGET_ARCH" == "x86_64" ]]; then
  # https://docs.bazel.build/versions/master/install-ubuntu.html
  PKG_NAME="bazel_${BAZEL_VERSION}-linux-x86_64.deb"
  DOWNLOAD_LINK="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${PKG_NAME}"
  SHA256SUM="08b71237eccc3c313e62976894fc260d9e1c1ecdfa5b14fc7477fce1c36c618c"
  download_if_not_cached $PKG_NAME $SHA256SUM $DOWNLOAD_LINK

  apt_get_update_and_install \
    zlib1g-dev

  # https://docs.bazel.build/versions/master/install-ubuntu.html#step-3-install-a-jdk-optional
  # openjdk-11-jdk

  dpkg -i "${PKG_NAME}"

  # Cleanup right after installation
  rm -rf "${PKG_NAME}"

  ## buildifier ##
  PKG_NAME="buildifier-${BUILDTOOLS_VERSION}.${TARGET_ARCH}.bin"
  CHECKSUM="f9a9c082b8190b9260fce2986aeba02a25d41c00178855a1425e1ce6f1169843"
  DOWNLOAD_LINK="https://github.com/bazelbuild/buildtools/releases/download/${BUILDTOOLS_VERSION}/buildifier"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  cp -f ${PKG_NAME} "${SYSROOT_DIR}/bin/buildifier"
  chmod a+x "${SYSROOT_DIR}/bin/buildifier"
  rm -rf ${PKG_NAME}

  info "Done installing bazel ${BAZEL_VERSION} with buildifier ${BUILDTOOLS_VERSION}"

elif [[ "$TARGET_ARCH" == "aarch64" ]]; then
  ARM64_BINARY="bazel-${BAZEL_VERSION}-linux-arm64"
  CHECKSUM="0797425b019c6ffb36e9290323d29563db93826d3abfa3cd784a39cb05ed4f61"
  DOWNLOAD_LINK="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${ARM64_BINARY}"
  # https://github.com/bazelbuild/bazel/releases/download/3.5.0/bazel-3.5.0-linux-arm64
  download_if_not_cached "${ARM64_BINARY}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
  cp -f ${ARM64_BINARY} "${SYSROOT_DIR}/bin/bazel"
  chmod a+x "${SYSROOT_DIR}/bin/bazel"
  rm -rf "${ARM64_BINARY}"

  cp /opt/apollo/rcfiles/bazel_completion.bash /etc/bash_completion.d/bazel

  PKG_NAME="buildifier-${BUILDTOOLS_VERSION}-linux-arm64"
  CHECKSUM="19d5b358cb099e264086b26091661fd7548df0a2400e47fd98238cfe0a3e67f9"
  DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  cp -f ${PKG_NAME} "${SYSROOT_DIR}/bin/buildifier"
  chmod a+x "${SYSROOT_DIR}/bin/buildifier"
  rm -rf ${PKG_NAME}

  info "Done installing bazel ${BAZEL_VERSION} with buildifier ${BUILDTOOLS_VERSION}"
else
  error "Target arch ${TARGET_ARCH} not supported yet"
  exit 1
fi

# Note(storypku):
# Used by `apollo.sh config` to determine native cuda compute capability.
bash ${CURR_DIR}/install_deviceQuery.sh

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
