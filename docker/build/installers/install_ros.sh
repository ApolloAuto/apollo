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

# Install dependencies.
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
    --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update -y && apt-get install -y \
    libbz2-dev \
    libconsole-bridge-dev \
    libeigen3-dev \
    libgstreamer-plugins-base0.10-dev \
    libgstreamer0.10-dev \
    liblog4cxx10-dev \
    liblz4-dev \
    libpoco-dev \
    libproj-dev \
    libtinyxml-dev \
    libyaml-cpp-dev \
    ros-indigo-catkin \
    sip-dev \
    uuid-dev \
    zlib1g-dev

# Set environment.
echo 'ROSCONSOLE_FORMAT=${file}:${line} ${function}() [${severity}] [${time}]: ${message}' >> /etc/profile

# Download apollo-ros.
VERSION=2.1.2
FILENAME=ros-indigo-apollo-${VERSION}-x86_64.tar.gz

mkdir -p /home/tmp
cd /home/tmp
wget https://github.com/ApolloAuto/apollo-platform/releases/download/${VERSION}/${FILENAME}
tar xzf ${FILENAME}

# Clean up.
rm -fr ${FILENAME} /etc/apt/sources.list.d/ros-latest.list
