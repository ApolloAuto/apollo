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

. /tmp/installers/installer_base.sh

ARCH=$(uname -m)

if [ "$ARCH" == "x86_64" ]; then
  # https://docs.bazel.build/versions/master/install-ubuntu.html
  VERSION="3.2.0"
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

elif [ "$ARCH" == "aarch64" ]; then
  BUILD=$1
  shift
  if [ "$BUILD" == "build" ]; then
    mkdir -p bazel
    pushd bazel
    wget https://github.com/bazelbuild/bazel/releases/download/0.5.3/bazel-0.5.3-dist.zip
    unzip bazel-0.5.3-dist.zip
    chmod a+w src/java_tools/buildjar/java/com/google/devtools/build/buildjar/javac/plugins/errorprone/ErrorPronePlugin.java
    wget https://apollocache.blob.core.windows.net/apollo-cache/ErrorPronePlugin.java.patch
    patch -p0 < ./ErrorPronePlugin.java.patch
    env EXTRA_BAZEL_ARGS="--host_javabase=@local_jdk//:jdk" bash ./compile.sh
    mv /tmp/installers/bazel/output/bazel /usr/local/bin/
    popd
  else
    wget https://apollocache.blob.core.windows.net/apollo-cache/bazel
    cp bazel /usr/local/bin/
    chmod a+x /usr/local/bin/bazel
  fi
else
    echo "not support $ARCH"
fi
