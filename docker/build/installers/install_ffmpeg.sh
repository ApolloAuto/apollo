#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

cd $( dirname "${BASH_SOURCE[0]}")

# References
# 1) https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
# 2) https://linuxize.com/post/how-to-install-ffmpeg-on-ubuntu-18-04
# 3) https://launchpad.net/~savoury1/+archive/ubuntu/ffmpeg4
# We choose 1) in this script
# cat > /etc/apt/sources.list.d/ffmpeg4.list <<EOF
# deb http://ppa.launchpad.net/savoury1/ffmpeg4/ubuntu bionic main
# EOF
# apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 374C7797FB006459

apt-get -y update && \
    apt-get -y install \
    nasm \
    yasm \
    libx265-dev \
    libnuma-dev

. /tmp/installers/installer_base.sh

VERSION="4.2.2"
PKG_NAME="ffmpeg-${VERSION}.tar.gz"
CHECKSUM="5447ca061444e574dc0d5e6da1657f49a64a0e660403995c7744beee3e69b2b8"
DOWNLOAD_LINK="https://github.com/FFmpeg/FFmpeg/archive/n${VERSION}.tar.gz"
# https://github.com/FFmpeg/FFmpeg/archive/n4.2.2.tar.gz
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf ${PKG_NAME} && mv "FFmpeg-n${VERSION}" ffmpeg

# Unused options
# --pkg-config-flags="--static"
DEST_DIR=/usr/local/ffmpeg4

pushd ffmpeg
    ./configure \
    --prefix=${DEST_DIR} \
    --extra-libs="-lpthread -lm" \
    --enable-shared \
    --enable-pic \
    --enable-gpl \
    --enable-libx265 \
    --enable-nonfree
    make -j$(nproc)
    make install
popd

rm -rf ${DEST_DIR}/share/man

echo "${DEST_DIR}/lib" > /etc/ld.so.conf.d/ffmpeg4.conf
ldconfig

rm -fr ${PKG_NAME} ffmpeg

apt-get -y update && \
    apt-get -y autoremove \
    nasm \
    yasm \
    libx265-dev

# Don't remove libnuma-dev!

