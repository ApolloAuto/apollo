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

apt-get update -y
apt-get install -y openjdk-8-jdk

# Install Bazel.
wget https://github.com/bazelbuild/bazel/releases/download/0.5.3/bazel-0.5.3-installer-linux-x86_64.sh
bash bazel-0.5.3-installer-linux-x86_64.sh

# Clean up.
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -fr bazel-0.5.3-installer-linux-x86_64.sh /etc/apt/sources.list.d/bazel.list
