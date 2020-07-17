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

cd "$(dirname "${BASH_SOURCE[0]}")"

. installer_base.sh

TARGET_ARCH=$(uname -m)

BAZEL_VERSION="3.4.1"

if [ "$TARGET_ARCH" == "x86_64" ]; then
  # https://docs.bazel.build/versions/master/install-ubuntu.html
  PKG_NAME="bazel_${BAZEL_VERSION}-linux-x86_64.deb"
  DOWNLOAD_LINK=https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${PKG_NAME}
  SHA256SUM="1a64c807716e10c872f1618852d95f4893d81667fe6e691ef696489103c9b460"
  download_if_not_cached $PKG_NAME $SHA256SUM $DOWNLOAD_LINK

  apt-get -y update && \
    apt-get -y install \
    zlib1g-dev

  # https://docs.bazel.build/versions/master/install-ubuntu.html#step-3-install-a-jdk-optional
  # openjdk-11-jdk

  dpkg -i $PKG_NAME

  ## buildifier ##
  PKG_NAME="buildifier"
  BUILDTOOLS_VERSION="3.3.0"
  CHECKSUM="0c5df005e2b65060c715a7c5764c2a04f7fac199bd73442e004e0bf29381a55a"
  DOWNLOAD_LINK="https://github.com/bazelbuild/buildtools/releases/download/${BUILDTOOLS_VERSION}/buildifier"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  chmod a+x ${PKG_NAME}
  cp -f ${PKG_NAME} "${SYSROOT_DIR}/bin"
  rm -rf ${PKG_NAME}

  ## buildozer
  PKG_NAME="buildozer"
  CHECKSUM="6618c2a4473ddc35a5341cf9a651609209bd5362e0ffa54413be256fe8a4081a"
  DOWNLOAD_LINK="https://github.com/bazelbuild/buildtools/releases/download/${BUILDOZER_VERSION}/buildozer"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  chmod a+x ${PKG_NAME}
  cp -f "${PKG_NAME}" "${SYSROOT_DIR}/bin/"
  rm -f "${PKG_NAME}"
  info "Done installing bazel ${BAZEL_VERSION} with buildifier and buildozer"

elif [ "$TARGET_ARCH" == "aarch64" ]; then
  ARM64_BINARY="bazel-${BAZEL_VERSION}-linux-arm64"
  CHECKSUM="07955cbef922b51025577df4e258d5dfc4f7adc5ec8ab110dedb411878d63627"
  DOWNLOAD_LINK="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${ARM64_BINARY}"
  # https://github.com/bazelbuild/bazel/releases/download/3.4.1/bazel-3.4.1-linux-arm64
  download_if_not_cached "${ARM64_BINARY}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
  chmod a+x ${ARM64_BINARY}
  cp -f ${ARM64_BINARY} "${SYSROOT_DIR}/bin/bazel"
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

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
