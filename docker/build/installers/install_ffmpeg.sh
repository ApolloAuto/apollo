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

# Reference https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
# Prepare
FFMPEG_HOME="$(cd $( dirname "${BASH_SOURCE[0]}" ); pwd)"
FFMPEG_HOME=$FFMPEG_HOME/ffmpeg
FFMPEG_SOURCE=$FFMPEG_HOME/ffmpeg_source
FFMPEG_BUILD=$FFMPEG_HOME/ffmpeg_build
FFMPEG_BIN=$FFMPEG_HOME/ffmpeg_bin
FFMPEG_TARGET=/usr/local/apollo/ffmpeg
mkdir -p $FFMPEG_SOURCE $FFMPEG_BUILD && cd $FFMPEG_SOURCE
apt-get update -y && apt-get install -y mercurial

wget https://www.nasm.us/pub/nasm/releasebuilds/2.14.02/nasm-2.14.02.tar.bz2
tar xjvf nasm-2.14.02.tar.bz2
HG_SETTING=$'[ui]\ntls = False'
HG_SETTING_FILE="/root/.hgrc"
echo "$HG_SETTING" > $HG_SETTING_FILE
if cd x265 2> /dev/null; then hg pull && hg update && cd ..; else hg clone https://bitbucket.org/multicoreware/x265; fi
rm $HG_SETTING_FILE
wget https://github.com/FFmpeg/FFmpeg/archive/n4.1.3.tar.gz
tar zxvf n4.1.3.tar.gz && mv FFmpeg-n4.1.3 ffmpeg

# Build and install
pushd nasm-2.14.02
  ./autogen.sh
  PATH="$FFMPEG_BIN:$PATH" ./configure --prefix="$FFMPEG_BUILD" --bindir="$FFMPEG_BIN"
  make
  make install
popd

pushd x265/build/linux
  PATH="$FFMPEG_BIN:$PATH" cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX="$FFMPEG_BUILD" -DENABLE_SHARED=off ../../source
  PATH="$FFMPEG_BIN:$PATH" make
  make install
popd

pushd ffmpeg
  PATH="$FFMPEG_BIN:$PATH" PKG_CONFIG_PATH="$PKG_CONFIG_PATH:$FFMPEG_BUILD/lib/pkgconfig" ./configure \
    --prefix="$FFMPEG_BUILD" \
    --pkg-config-flags="--static" \
    --extra-cflags="-I$FFMPEG_BUILD/include" \
    --extra-ldflags="-L$FFMPEG_BUILD/lib" \
    --extra-libs="-lpthread -lm -lx265" \
    --bindir="$FFMPEG_BIN" \
    --enable-shared \
    --disable-stripping \
    --enable-pic \
    --enable-gpl \
    --enable-libx265 \
    --enable-nonfree
  PATH="$FFMPEG_BIN:$PATH" make
  make install
popd

mkdir -p $FFMPEG_TARGET
cp -r $FFMPEG_BUILD/include $FFMPEG_TARGET/include
cp -r $FFMPEG_BUILD/lib $FFMPEG_TARGET/lib

# Clean
rm -fr $FFMPEG_HOME
apt-get autoremove -y mercurial
