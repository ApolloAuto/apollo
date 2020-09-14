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

BAZEL_VERSION="3.4.1"

if [ "$TARGET_ARCH" == "x86_64" ]; then
  # https://docs.bazel.build/versions/master/install-ubuntu.html
  PKG_NAME="bazel_${BAZEL_VERSION}-linux-x86_64.deb"
  DOWNLOAD_LINK="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${PKG_NAME}"
  SHA256SUM="dc8f51b7ed039d57bb990a1eebddcbb0014fe267a88df8972f4609ded1f11c90"
  download_if_not_cached $PKG_NAME $SHA256SUM $DOWNLOAD_LINK

  apt_get_update_and_install \
    zlib1g-dev

  # https://docs.bazel.build/versions/master/install-ubuntu.html#step-3-install-a-jdk-optional
  # openjdk-11-jdk

  dpkg -i "${PKG_NAME}"

  # Cleanup right after installation
  rm -rf "${PKG_NAME}"

  ## buildifier ##
  BUILDTOOLS_VERSION="3.4.0"
  PKG_NAME="buildifier-${BUILDTOOLS_VERSION}.${TARGET_ARCH}.bin"
  CHECKSUM="5d47f5f452bace65686448180ff63b4a6aaa0fb0ce0fe69976888fa4d8606940"
  DOWNLOAD_LINK="https://github.com/bazelbuild/buildtools/releases/download/${BUILDTOOLS_VERSION}/buildifier"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  cp -f ${PKG_NAME} "${SYSROOT_DIR}/bin/buildifier"
  chmod a+x "${SYSROOT_DIR}/bin/buildifier"
  rm -rf ${PKG_NAME}

  info "Done installing bazel ${BAZEL_VERSION} with buildifier ${BUILDTOOLS_VERSION}"

elif [ "$TARGET_ARCH" == "aarch64" ]; then
  ARM64_BINARY="bazel-${BAZEL_VERSION}-linux-arm64"
  CHECKSUM="07955cbef922b51025577df4e258d5dfc4f7adc5ec8ab110dedb411878d63627"
  DOWNLOAD_LINK="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${ARM64_BINARY}"
  # https://github.com/bazelbuild/bazel/releases/download/3.4.1/bazel-3.4.1-linux-arm64
  download_if_not_cached "${ARM64_BINARY}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
  cp -f ${ARM64_BINARY} "${SYSROOT_DIR}/bin/bazel"
  chmod a+x "${SYSROOT_DIR}/bin/bazel"
  rm -rf "${ARM64_BINARY}"

  cp /opt/apollo/rcfiles/bazel_completion.bash /etc/bash_completion.d/bazel

  BUILDTOOLS_VERSION="3.3.0"
  PKG_NAME="buildifier-${BUILDTOOLS_VERSION}-linux-arm64"
  CHECKSUM="11df20761f6a14adcc21ea684225e029d6a5f4a881eb3477ea8c24afda316bdf"
  DOWNLOAD_LINK="https://apollo-platform-system.bj.bcebos.com/archive/6.0/${PKG_NAME}"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  cp -f ${PKG_NAME} "${SYSROOT_DIR}/bin/buildifier"
  chmod a+x "${SYSROOT_DIR}/bin/buildifier"
  rm -rf ${PKG_NAME}
  # buildozer can be retrieved from
  # https://apollo-platform-system.bj.bcebos.com/archive/6.0/buildozer-3.3.0-linux-arm64

  info "Done installing bazel ${BAZEL_VERSION} with buildifier ${BUILDTOOLS_VERSION}"
else
  error "Target arch ${TARGET_ARCH} not supported yet"
  exit 1
fi

# Note(storypku):
# Used by `apollo.sh config` to determine native cuda compute capability.
bash ${CURR_DIR}/install_device_query.sh

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
