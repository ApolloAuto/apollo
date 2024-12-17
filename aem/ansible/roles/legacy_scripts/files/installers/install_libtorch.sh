#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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

INSTALL_ATOM="${INSTALL_ATOM:-libtorch-1.7.0-r2}"

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/general_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/general_package_install_funcs.sh

if ldconfig -p | grep -q libtorch; then
  info "libtorch was already installed"
  exit 0
fi

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

# bash ${CURR_DIR}/install_mkl.sh

TARGET_ARCH="$(uname -m)"

src_prepare_pre() {
  apt_get_update_and_install \
    libopenblas-base \
    libopenmpi-dev \
    libomp-dev
  pip3 install Cython
}
# sudo apt update && sudo apt install -y libopenblas-base libopenmpi-dev libomp-dev

# pip3 install Cython

##============================================================##
# libtorch_cpu

determine_gpu_use_host
MY_PVR="${PV}"
if [[ "${PR}" != "r0" ]]; then
  MY_PVR="${PV}-${PR:1}"
fi
# TODO: support custom cuda version
if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
  if [[ -z "${SRC_URI}" ]]; then
    SRC_URI="https://apollo-system.cdn.bcebos.com/archive/6.0/${PN}_cpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz"
    if [[ "${USE_NVIDIA_GPU}" -eq 1 ]]; then
      SRC_URI="${SRC_URI}
      https://apollo-system.cdn.bcebos.com/archive/6.0/${PN}_gpu-${MY_PVR}-cu111-linux-${TARGET_ARCH}.tar.gz -> ${PN}_gpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz
      "
    elif [[ "${USE_AMD_GPU}" -eq 1 ]]; then
      # SRC_URI="${SRC_URI}
      # https://apollo-system.cdn.bcebos.com/archive/6.0/${PN}_gpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz
      # "
      : # use custom download method
    fi
  fi
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
  if [[ -z "${SRC_URI}" ]]; then
    SRC_URI="
    https://apollo-pkg-beta.cdn.bcebos.com/archive/${PN}_cpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz
    "
    if [[ "${USE_NVIDIA_GPU}" -eq 1 ]]; then
      SRC_URI="${SRC_URI}
      https://apollo-pkg-beta.cdn.bcebos.com/archive/${PN}_gpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz
      "
    elif [[ "${USE_AMD_GPU}" -eq 1 ]]; then
      # SRC_URI="${SRC_URI}
      # https://apollo-pkg-beta.cdn.bcebos.com/archive/${PN}_gpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz
      # "
      error "AMD libtorch for ${TARGET_ARCH} not ready. Exiting..."
      exit 1
    fi
  fi
else
  error "libtorch for ${TARGET_ARCH} not ready. Exiting..."
  exit 1
fi

# if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
#   # https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
#   VERSION="1.7.0-2"
#   CHECKSUM="02fd4f30e97ce8911ef933d0516660892392e95e6768b50f591f4727f6224390"
#   PKG_NAME="libtorch_cpu-${VERSION}-linux-${TARGET_ARCH}.tar.gz"
#   DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
# elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
#   VERSION="1.11.0"
#   CHECKSUM="7faae6caad3c7175070263a0767732c0be9a92be25f9fb022aebe14a4cd2d092"
#   PKG_NAME="libtorch_cpu-${VERSION}-linux-${TARGET_ARCH}.tar.gz"
#   DOWNLOAD_LINK="https://apollo-pkg-beta.cdn.bcebos.com/archive/${PKG_NAME}"
# else
#   error "libtorch for ${TARGET_ARCH} not ready. Exiting..."
#   exit 1
# fi

src_prepare() {
  general_src_prepare
  # download amd libtorch
  if [[ "${USE_AMD_GPU}" -eq 1 ]]; then
    fileid="1UMzACmxzZD8KVitEnSk-BhXa38Kkl4P-"
    uri="https://docs.google.com/uc?export=download&id=${fileid}"
    # distpath="${DISTDIR}/libtorch_amd.tar.gz"
    distpath="${DISTDIR}/libtorch_amd-${fileid}.tar.gz"
    wget --load-cookies ${T}/cookies.txt \
      "https://docs.google.com/uc?export=download&confirm=
            $(wget --quiet --save-cookies ${T}/cookies.txt --keep-session-cookies \
        --no-check-certificate ${uri} \
        -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${fileid}" \
      -O "${distpath}" && rm -rf ${T}/cookies.txt
  fi
}

# download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

src_unpack() {
  # libtorch_cpu
  mkdir -p "${WORKDIR}/${PN}_cpu"
  tar -xf "${DISTDIR}/${PN}_cpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz" -C "${WORKDIR}/${PN}_cpu" --strip-components=1
  if [[ "${USE_NVIDIA_GPU}" -eq 1 ]]; then
    # libtorch_gpu of nvidia
    mkdir -p "${WORKDIR}/${PN}_gpu"
    tar -xf "${DISTDIR}/${PN}_gpu-${MY_PVR}-linux-${TARGET_ARCH}.tar.gz" -C "${WORKDIR}/${PN}_gpu" --strip-components=1
  elif [[ "${USE_AMD_GPU}" -eq 1 ]]; then
    # libtorch_gpu of amd
    mkdir -p "${WORKDIR}/${PN}_amd"
    fileid="1UMzACmxzZD8KVitEnSk-BhXa38Kkl4P-"
    tar -xf "${DISTDIR}/libtorch_amd-${fileid}.tar.gz" -C "${WORKDIR}/${PN}_amd" --strip-components=1
  fi
}

src_configure() {
  : # do nothing
}

src_compile() {
  : # do nothing
}

src_install() {
  # libtorch_cpu
  cp -arpv "${WORKDIR}/${PN}_cpu" ${INSTALL_PREFIX}/libtorch_cpu
  if [[ "${USE_NVIDIA_GPU}" -eq 1 ]]; then
    # libtorch_gpu of nvidia
    cp -arpv "${WORKDIR}/${PN}_gpu" ${INSTALL_PREFIX}/libtorch_gpu
  elif [[ "${USE_AMD_GPU}" -eq 1 ]]; then
    # libtorch_gpu of amd
    # TODO: check if the following commands are correct
    cp -arpv "${WORKDIR}/${PN}_amd/libtorch" ${INSTALL_PREFIX}/libtorch_gpu
    cp -arpv "${WORKDIR}/${PN}_amd/libtorch_deps/libamdhip64.so.4" /opt/rocm/hip/lib/
    cp -arpv "${WORKDIR}/${PN}_amd/libtorch_deps/libmagma.so" /opt/apollo/sysroot/lib/
    cp -arpv "${WORKDIR}/${PN}_amd/libtorch_deps/"* ${INSTALL_PREFIX}/lib/
  fi
}

# tar xzf "${PKG_NAME}"
# mv libtorch_cpu /usr/local/libtorch_cpu
# rm -f "${PKG_NAME}"
# ok "Successfully installed libtorch_cpu ${VERSION}"
#
# ##============================================================##
# # libtorch_gpu
# determine_gpu_use_host
# if [[ "${USE_NVIDIA_GPU}" -eq 1 ]]; then
#   # libtorch_gpu nvidia
#   if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
#     VERSION="1.7.0-2"
#     CHECKSUM="b64977ca4a13ab41599bac8a846e8782c67ded8d562fdf437f0e606cd5a3b588"
#     PKG_NAME="libtorch_gpu-${VERSION}-cu111-linux-x86_64.tar.gz"
#     DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
#   else # AArch64
#     VERSION="1.11.0"
#     PKG_NAME="libtorch_gpu-${VERSION}-linux-${TARGET_ARCH}.tar.gz"
#     CHECKSUM="661346303cafc832ef2d37b734ee718c85cdcf5ab21b72b13ef453cb40f13f86"
#     DOWNLOAD_LINK="https://apollo-pkg-beta.cdn.bcebos.com/archive/${PKG_NAME}"
#   fi
#
#   download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
#
#   tar xzf "${PKG_NAME}"
#   mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_gpu/
# elif [[ "${USE_AMD_GPU}" -eq 1 ]]; then
#   if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
#     PKG_NAME="libtorch_amd.tar.gz"
#     FILE_ID="1UMzACmxzZD8KVitEnSk-BhXa38Kkl4P-"
#   else # AArch64
#     error "AMD libtorch for ${TARGET_ARCH} not ready. Exiting..."
#     exit 1
#   fi
#   DOWNLOAD_LINK="https://docs.google.com/uc?export=download&id=${FILE_ID}"
#   wget --load-cookies /tmp/cookies.txt \
#     "https://docs.google.com/uc?export=download&confirm=
#             $(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies \
#       --no-check-certificate ${DOWNLOAD_LINK} \
#       -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${FILE_ID}" \
#     -O "${PKG_NAME}" && rm -rf /tmp/cookies.txt
#
#   tar xzf "${PKG_NAME}"
#   mv "${PKG_NAME%.tar.gz}/libtorch" /usr/local/libtorch_gpu/
#   mv "${PKG_NAME%.tar.gz}/libtorch_deps/libamdhip64.so.4" /opt/rocm/hip/lib/
#   mv "${PKG_NAME%.tar.gz}/libtorch_deps/libmagma.so" /opt/apollo/sysroot/lib/
#   mv "${PKG_NAME%.tar.gz}/libtorch_deps/"* /usr/local/lib/
#   rm -r "${PKG_NAME%.tar.gz}"
#   ldconfig
# fi
#
# # Cleanup
# rm -f "${PKG_NAME}"

pkg_install_post() {
  ok "Successfully installed libtorch_cpu ${PV}"
  ok "Successfully installed libtorch_gpu ${PV}"
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi
