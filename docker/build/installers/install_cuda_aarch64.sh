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

# Install cuda8
wget http://123.57.58.164/apollo-docker/cuda8_cudnn6/cuda-repo-l4t-8-0-local_8.0.84-1_arm64.deb
dpkg -i cuda-repo-l4t-8-0-local_8.0.84-1_arm64.deb

# Install toolkit
apt-get update -y
apt-get -y install cuda-toolkit-8-0

#Install cudnn7
wget http://123.57.58.164/apollo-docker/cuda8_cudnn6/libcudnn6_6.0.21-1+cuda8.0_arm64.deb
dpkg -i libcudnn6_6.0.21-1+cuda8.0_arm64.deb

#Install cudnn7-dev
wget http://123.57.58.164/apollo-docker/cuda8_cudnn6/libcudnn6-dev_6.0.21-1+cuda8.0_arm64.deb
dpkg -i libcudnn6-dev_6.0.21-1+cuda8.0_arm64.deb


# Clean up.
rm -fr libcudnn6-dev_6.0.21-1+cuda8.0_arm64.deb libcudnn6_6.0.21-1+cuda8.0_arm64.deb cuda-repo-l4t-8-0-local_8.0.84-1_arm64.deb
