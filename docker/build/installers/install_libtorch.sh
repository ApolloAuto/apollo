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

set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# TODO(build): Docs on how to build libtorch on Jetson boards
# References:
#   https://github.com/ApolloAuto/apollo/blob/pre6/docker/build/installers/install_libtorch.sh
#   https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.pytorch
#   https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-6-0-now-available
#   https://github.com/pytorch/pytorch/blob/master/docker/caffe2/ubuntu-16.04-cpu-all-options/Dockerfile
#
# Following content describes how to build libtorch source on orin:
# 1. Downloading libtorch source: git clone https://github.com/pytorch/pytorch.git
# 2. install py deps: pip3 install --no-cache-dir PyYAML typing
# 3. init sub modules: git clone --recursive --single-branch --branch apollo --depth 1 https://github.com/pytorch/pytorch.git && git checkout release/1.11
# 4. set env: export USE_CUDA=1 && export TORCH_CUDA_ARCH_LIST="3.5;5.0;5.2;6.1;7.0;7.5;8.6;8.7" && export BUILD_CAFFE2=1 && export USE_NCCL=0
# 5. python3 setup.py install
# 6. mkdir libtorch_gpu && cp -r include libtorch_gpu/ && cp -r lib libtorch_gpu/ && sudo mv libtorch_gpu /usr/local/

bash ${CURR_DIR}/install_mkl.sh

TARGET_ARCH="$(uname -m)"

sudo apt update && sudo apt install -y libopenblas-base libopenmpi-dev libomp-dev

pip3 install Cython

##============================================================##
# libtorch_cpu

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
  # https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
  VERSION="1.7.0-2"
  CHECKSUM="02fd4f30e97ce8911ef933d0516660892392e95e6768b50f591f4727f6224390"
  PKG_NAME="libtorch_cpu-${VERSION}-linux-${TARGET_ARCH}.tar.gz"
  DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
  VERSION="1.11.0"
  CHECKSUM="7faae6caad3c7175070263a0767732c0be9a92be25f9fb022aebe14a4cd2d092"
  PKG_NAME="libtorch_cpu-${VERSION}-linux-${TARGET_ARCH}.tar.gz"
  DOWNLOAD_LINK="https://apollo-pkg-beta.cdn.bcebos.com/archive/${PKG_NAME}"
else
  error "libtorch for ${TARGET_ARCH} not ready. Exiting..."
  exit 1
fi

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"
mv libtorch_cpu /usr/local/libtorch_cpu
rm -f "${PKG_NAME}"
ok "Successfully installed libtorch_cpu ${VERSION}"

##============================================================##
# libtorch_gpu
determine_gpu_use_host
if [[ "${USE_NVIDIA_GPU}" -eq 1 ]]; then
  # libtorch_gpu nvidia
  if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    VERSION="1.7.0-2"
    CHECKSUM="b64977ca4a13ab41599bac8a846e8782c67ded8d562fdf437f0e606cd5a3b588"
    PKG_NAME="libtorch_gpu-${VERSION}-cu111-linux-x86_64.tar.gz"
    DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
  else # AArch64
    VERSION="1.11.0"
    PKG_NAME="libtorch_gpu-${VERSION}-linux-${TARGET_ARCH}.tar.gz"
    CHECKSUM="661346303cafc832ef2d37b734ee718c85cdcf5ab21b72b13ef453cb40f13f86"
    DOWNLOAD_LINK="https://apollo-pkg-beta.cdn.bcebos.com/archive/${PKG_NAME}"
  fi

  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  tar xzf "${PKG_NAME}"
  mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_gpu/
elif [[ "${USE_AMD_GPU}" -eq 1 ]]; then
  if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    PKG_NAME="libtorch_amd.tar.gz"
    FILE_ID="1UMzACmxzZD8KVitEnSk-BhXa38Kkl4P-"
  else # AArch64
    error "AMD libtorch for ${TARGET_ARCH} not ready. Exiting..."
    exit 1
  fi
  DOWNLOAD_LINK="https://docs.google.com/uc?export=download&id=${FILE_ID}"
  wget --load-cookies /tmp/cookies.txt \
    "https://docs.google.com/uc?export=download&confirm=
            $(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies \
      --no-check-certificate ${DOWNLOAD_LINK} \
      -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${FILE_ID}" \
    -O "${PKG_NAME}" && rm -rf /tmp/cookies.txt

  tar xzf "${PKG_NAME}"
  mv "${PKG_NAME%.tar.gz}/libtorch" /usr/local/libtorch_gpu/
  mv "${PKG_NAME%.tar.gz}/libtorch_deps/libamdhip64.so.4" /opt/rocm/hip/lib/
  mv "${PKG_NAME%.tar.gz}/libtorch_deps/libmagma.so" /opt/apollo/sysroot/lib/
  mv "${PKG_NAME%.tar.gz}/libtorch_deps/"* /usr/local/lib/
  rm -r "${PKG_NAME%.tar.gz}"
  ldconfig
fi

# Cleanup
rm -f "${PKG_NAME}"
ok "Successfully installed libtorch_gpu ${VERSION}"
