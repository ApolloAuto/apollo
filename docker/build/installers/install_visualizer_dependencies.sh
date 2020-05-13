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
    freeglut3-dev \
    mesa-common-dev \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libconsole-bridge-dev

# See Ref:
# https://hub.docker.com/r/nvidia/cudagl
# https://gitlab.com/nvidia/container-images/opengl/blob/ubuntu18.04/glvnd/devel/Dockerfile
# https://www.pugetsystems.com/labs/hpc/NVIDIA-Docker2-with-OpenGL-and-X-Display-Output-1527/

# DON'T INSTALL THESE!!!
# libnvidia-gl-440 # trouble maker for `nvidia-smi`

# For test run
# /usr/local/cuda/samples/5_Simulations/nbody

apt-get clean && \
    rm -rf /var/lib/apt/lists/*

bash /tmp/installers/install_qt.sh
