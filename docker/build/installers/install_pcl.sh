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

WORKHORSE="$1"
if [ -z "${WORKHORSE}" ]; then
    WORKHORSE="cpu"
fi

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# Install system-provided pcl
# apt-get -y update && \
#   apt-get -y install \
#   libpcl-dev
# exit 0
if ldconfig -p | grep -q libpcl_common ; then
    info "Found existing PCL installation. Skipp re-installation."
    exit 0
fi

GPU_OPTIONS="-DCUDA_ARCH_BIN=\"${SUPPORTED_NVIDIA_SMS}\""
if [ "${WORKHORSE}" = "cpu" ]; then
    GPU_OPTIONS="-DWITH_CUDA=OFF"
fi

info "GPU Options for PCL:\"${GPU_OPTIONS}\""

TARGET_ARCH="$(uname -m)"
ARCH_OPTIONS=""
if [ "${TARGET_ARCH}" = "x86_64" ]; then
    ARCH_OPTIONS="-DPCL_ENABLE_SSE=ON"
else
    ARCH_OPTIONS="-DPCL_ENABLE_SSE=OFF"
fi

# libpcap-dev
# libopenmpi-dev
# libboost-all-dev

apt_get_update_and_install \
    libeigen3-dev \
    libflann-dev \
    libglew-dev \
    libglfw3-dev \
    freeglut3-dev \
    libusb-1.0-0-dev \
    libdouble-conversion-dev \
    libopenni-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    liblz4-dev \
    libfreetype6-dev \
    libpcap-dev \
    libqhull-dev

if [[ -d "/usr/include/boost" ]]; then
    rm -rf /usr/include/boost
    cp -r /opt/apollo/sysroot/include/boost /usr/include/boost
fi

# NOTE(storypku)
# libglfw3-dev depends on libglfw3,
# and libglew-dev have a dependency over libglew2.0

THREAD_NUM=$(nproc)

# VERSION="1.11.0"
# CHECKSUM="4255c3d3572e9774b5a1dccc235711b7a723197b79430ef539c2044e9ce65954" # 1.11.0

VERSION="1.10.1"
CHECKSUM="61ec734ec7c786c628491844b46f9624958c360012c173bbc993c5ff88b4900e" # 1.10.1
PKG_NAME="pcl-${VERSION}.tar.gz"

DOWNLOAD_LINK="https://github.com/PointCloudLibrary/pcl/archive/${PKG_NAME}"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
tar xzf ${PKG_NAME}

# Ref: https://src.fedoraproject.org/rpms/pcl.git
#  -DPCL_PKGCONFIG_SUFFIX:STRING="" \
#  -DCMAKE_SKIP_RPATH=ON \

pushd pcl-pcl-${VERSION}/
    # disable sse patch for avoiding eigen core dump
    # patch -p1 < ${CURR_DIR}/pcl-sse-fix-${VERSION}.patch
    mkdir build && cd build
    cmake .. \
        "${GPU_OPTIONS}" \
        "${ARCH_OPTIONS}" \
        -DPCL_ENABLE_SSE=ON \
        -DWITH_DOCS=OFF \
        -DWITH_TUTORIALS=OFF \
        -DBUILD_global_tests=OFF \
        -DOPENNI_INCLUDE_DIR:PATH=/usr/include/ni \
        -DBoost_NO_SYSTEM_PATHS=TRUE \
        -DBOOST_ROOT:PATHNAME="${SYSROOT_DIR}" \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
        -DCMAKE_BUILD_TYPE=Release
    make -j${THREAD_NUM}
    make install
popd

ldconfig

ok "Successfully installed PCL ${VERSION}"

# Clean up
rm -fr ${PKG_NAME} pcl-pcl-${VERSION}

if [[ -n "${CLEAN_DEPS}" ]]; then
    # Remove build-deps for PCL
    # Note(storypku):
    # Please keep libflann-dev as it was required by local_config_pcl
    apt_get_remove \
        libeigen3-dev \
        libglew-dev \
        libglfw3-dev \
        freeglut3-dev \
        libusb-1.0-0-dev \
        libdouble-conversion-dev \
        libopenni-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        liblz4-dev \
        libfreetype6-dev \
        libpcap-dev \
        libqhull-dev

    # Add runtime-deps for pcl
    apt_get_update_and_install \
        libusb-1.0-0 \
        libopenni0 \
        libfreetype6 \
        libtiff5 \
        libdouble-conversion1 \
        libpcap0.8 \
        libqhull7
fi

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
