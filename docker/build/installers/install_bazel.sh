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
VERSION="3.3.0"

if [ "$TARGET_ARCH" == "x86_64" ]; then
  # https://docs.bazel.build/versions/master/install-ubuntu.html
  PKG_NAME="bazel_${VERSION}-linux-x86_64.deb"
  DOWNLOAD_LINK=https://github.com/bazelbuild/bazel/releases/download/${VERSION}/${PKG_NAME}
  SHA256SUM="215b160b363fb88dd8b73035bf842819f147c6a7d81e4f0bde89310328712973"
  download_if_not_cached $PKG_NAME $SHA256SUM $DOWNLOAD_LINK

  apt-get -y update && \
    apt-get -y install \
    zlib1g-dev

  # https://docs.bazel.build/versions/master/install-ubuntu.html#step-3-install-a-jdk-optional
  # openjdk-11-jdk

  dpkg -i $PKG_NAME

  ## buildifier ##
  PKG_NAME="buildifier"
  CHECKSUM="0c5df005e2b65060c715a7c5764c2a04f7fac199bd73442e004e0bf29381a55a"
  DOWNLOAD_LINK="https://github.com/bazelbuild/buildtools/releases/download/${VERSION}/buildifier"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  chmod a+x ${PKG_NAME}
  cp -f ${PKG_NAME} "${SYSROOT_DIR}/bin"
  rm -f ${PKG_NAME}

  ## buildozer
  PKG_NAME="buildozer"
  CHECKSUM="6618c2a4473ddc35a5341cf9a651609209bd5362e0ffa54413be256fe8a4081a"
  DOWNLOAD_LINK="https://github.com/bazelbuild/buildtools/releases/download/${VERSION}/buildozer"
  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  chmod a+x ${PKG_NAME}
  cp ${PKG_NAME} "${SYSROOT_DIR}/bin"
  rm -rf ${PKG_NAME}
  info "Done installing bazel ${VERSION} with buildifier and buildozer"

elif [ "$TARGET_ARCH" == "aarch64" ]; then
  INSTALL_MODE="$1"
  # Ref: https://docs.bazel.build/versions/master/install-compile-source.html
  # Ref: https://github.com/storypku/storydev/blob/master/bazel-build/build-bazel-from-source.md
  if [[ "${INSTALL_MODE}" == "build" ]]; then
    apt-get -y update && \
      apt-get -y install \
      build-essential openjdk-11-jdk python3 zip unzip

    if [[ ! -e /usr/bin/python ]]; then
        ln -s /usr/bin/python3 /usr/local/bin/python
    fi

    PKG_NAME="bazel-${VERSION}-dist.zip"
    CHECKSUM="44ec129436f6de45f2230e14100104919443a1364c2491f5601666b358738bfa"
    DOWNLOAD_LINK="https://github.com/bazelbuild/bazel/releases/download/${VERSION}/${PKG_NAME}"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    BBUILD_DIR="${PKG_NAME%.zip}"
    unzip "${PKG_NAME}" -d "${BBUILD_DIR}"

    pushd ${BBUILD_DIR}
      # env EXTRA_BAZEL_ARGS="--host_javabase=@local_jdk//:jdk" bash ./compile.sh
      env SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH}" bash ./compile.sh
      cp -f output/bazel ${SYSROOT_DIR}/bin/
      chmod a+x ${SYSROOT_DIR}/bin
    popd
    rm -rf "${PKG_NAME}" "${BBUILD_DIR}"
  else # Download Mode
    PKG_NAME="bazel-${VERSION}-aarch64-linux-gnu.tar.gz"
    DOWNLOAD_LINK="https://apollo-platform-system.bj.bcebos.com/archive/6.0/${PKG_NAME}"
    CHECKSUM="56b904a06a809da59c0a20ccd570f51d0b9d9daa4cf551a73357ffd0a09d61d0"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xvf "${PKG_NAME}"
    pushd "bazel-${VERSION}-aarch64-linux-gnu"
        DEST=${SYSROOT_DIR} bash install.sh
    popd
    rm -rf "bazel-${VERSION}-aarch64-linux-gnu" "${PKG_NAME}"
  fi
else
  error "Target arch ${TARGET_ARCH} not supported yet"
fi

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
