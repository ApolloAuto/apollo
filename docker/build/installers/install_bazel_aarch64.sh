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

# Install Bazel.
cd "$(dirname "${BASH_SOURCE[0]}")"

wget http://123.57.58.164/apollo-docker/jdk-8u144-linux-arm64-vfp-hflt.tar.gz
tar zxvf jdk-8u144-linux-arm64-vfp-hflt.tar.gz
sudo rm -rf /usr/lib/java
sudo mkdir /usr/lib/java
sudo cp -r jdk1.8.0_144/* /usr/lib/java/

sudo echo -e "export JAVA_HOME=/usr/lib/java\n\
export JRE_HOME=\${JAVA_HOME}/jre\n\
export CLASSPATH=.:\${JAVA_HOME}/lib:\${JRE_HOME}/lib" >> /etc/skel/.bashrc

wget http://123.57.58.164/apollo-docker/bazel_aarch64.zip
unzip bazel_aarch64.zip
cp ./bazel/bazel /usr/local/bazel
ln -s /usr/local/bazel /usr/bin/bazel
chmod +x /usr/bin/bazel

# Clean up.
rm -fr bazel_aarch64.zip bazel jdk-8u144-linux-arm64-vfp-hflt.tar.gz jdk1.8.0_144
