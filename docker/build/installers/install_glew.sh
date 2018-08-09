#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

# Install OpenGL
echo "deb http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | \
    sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
echo "deb-src http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | \
    sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
apt-get update -y
apt-get install -y --force-yes libglfw3 libglfw3-dev freeglut3-dev

# Install GLEW
wget https://github.com/nigels-com/glew/releases/download/glew-2.0.0/glew-2.0.0.zip
unzip glew-2.0.0.zip
pushd glew-2.0.0
make -j8
make install
popd

ln -s /usr/lib64/libGLEW.so /usr/lib/libGLEW.so
ln -s /usr/lib64/libGLEW.so.2.0 /usr/lib/libGLEW.so.2.0

# Clean up.
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -fr glew-2.0.0.zip glew-2.0.0 /etc/apt/sources.list.d/fillwave_ext.list
