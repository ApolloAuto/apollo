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
. ./installer_base.sh

# References
# 1) http://www.linuxfromscratch.org/blfs/view/svn/multimedia/ffmpeg.html
# 2) https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
# 3) https://linuxize.com/post/how-to-install-ffmpeg-on-ubuntu-18-04
# 4) https://launchpad.net/~savoury1/+archive/ubuntu/ffmpeg4
# We choose 1) in this script
# cat > /etc/apt/sources.list.d/ffmpeg4.list <<EOF
# deb http://ppa.launchpad.net/savoury1/ffmpeg4/ubuntu bionic main
# EOF
# apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 374C7797FB006459

apt_get_update_and_install \
    nasm \
    yasm \
    libx265-dev \
    libass-dev \
    libfdk-aac-dev \
    libmp3lame-dev \
    libopus-dev \
    libtheora-dev \
    libvorbis-dev \
    libvpx-dev \
    libx264-dev \
    libnuma-dev

VERSION="4.3.1"
PKG_NAME="ffmpeg-${VERSION}.tar.xz"
CHECKSUM="ad009240d46e307b4e03a213a0f49c11b650e445b1f8be0dda2a9212b34d2ffb"
DOWNLOAD_LINK="http://ffmpeg.org/releases/ffmpeg-${VERSION}.tar.xz"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xJf ${PKG_NAME}
pushd ffmpeg-${VERSION}
    sed -i 's/-lflite"/-lflite -lasound"/' configure
    ./configure \
        --prefix=${SYSROOT_DIR} \
        --extra-libs="-lpthread -lm" \
        --enable-gpl        \
        --enable-version3   \
        --enable-nonfree    \
        --disable-static    \
        --enable-shared     \
        --disable-debug     \
        --enable-avresample \
        --enable-libass     \
        --enable-libfdk-aac \
        --enable-libfreetype \
        --enable-libmp3lame \
        --enable-libopus    \
        --enable-libtheora  \
        --enable-libvorbis  \
        --enable-libvpx     \
        --enable-libx264    \
        --enable-libx265    \
        --enable-nonfree
    make -j$(nproc)
    make install
popd

ldconfig

rm -fr ${PKG_NAME} ffmpeg-${VERSION}

if [[ -n "${CLEAN_DEPS}" ]]; then
    apt_get_remove \
        nasm \
        yasm \
        libx265-dev \
        libass-dev \
        libfdk-aac-dev \
        libmp3lame-dev \
        libopus-dev \
        libtheora-dev \
        libvorbis-dev \
        libvpx-dev \
        libx264-dev

    # Don't remove libnuma-dev as it is required by coinor-libipopt1v5

    # install runtime-dependencies of ffmpeg
    apt_get_update_and_install \
        libvpx5 \
        libx264-152 \
        libx265-146 \
        libopus0   \
        libmp3lame0 \
        libvorbis0a \
        libvorbisenc2 \
        libfdk-aac1 \
        libass9     \
        libtheora0
fi

