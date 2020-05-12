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

apt-get -y update && \
    apt-get -y install \
    libproj-dev

# https://github.com/OSGeo/proj.4/archive/4.9.3.zip
#. /tmp/installers/installer_base.sh
#
#VERSION="4.9.3"
#PKG_NAME="proj.4-${VERSION}.zip"
#CHECKSUM="9d6d845ae77928441631882e25177117534dbe4311b823ee35eb100d3b69a78e"
#DOWNLOAD_LINK="https://github.com/OSGeo/proj.4/archive/4.9.3.zip"
#
#download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"
#
#unzip $PKG_NAME
#
#MY_DEST_DIR=/usr/local/proj4
#
## works
#pushd PROJ-${VERSION}
#mkdir build && cd build
#
#cmake .. -DCMAKE_INSTALL_PREFIX=$MY_DEST_DIR
#make -j`nproc`
#make install
#cd ..
#
#cp COPYING $MY_DEST_DIR/
#popd
#
#ok "Successfully built proj4. version=$VERSION"
#
#export LD_LIBRARY_PATH=$MY_DEST_DIR/lib:$LD_LIBRARY_PATH
#
## clean up.
#rm -fr $PKG_NAME PROJ-$VERSION
