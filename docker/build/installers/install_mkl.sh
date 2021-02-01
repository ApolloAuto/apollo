#!/usr/bin/env bash

###############################################################################
# Copyright 2021 The Apollo Authors. All Rights Reserved.
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

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" != "x86_64" ]]; then
    exit 0
fi

pip3_install mkl==2021.1.1

# Workaround for removing duplicate entries in mkl PYPI installation
function mkl_relink {
    MKL_LIBDIR="/usr/local/lib"
    for so in ${MKL_LIBDIR}/libmkl*.so; do
        so1="${so}.1"

        if [[ "$(basename ${so})" == "libmkl_sycl.so" ]]; then
            rm -f ${so} ${so1} || true
            continue
        fi

        if [[ -f "${so}.1" ]]; then
            cs1=$(sha256sum ${so1} | awk '{print $1}')
            cs0=$(sha256sum ${so} | awk '{print $1}')
            if [[ "${cs1}" == "${cs0}" ]]; then
                so1_name="$(basename $so1)"
                warning "Duplicate so ${so} with ${so1_name} found, re-symlinking..."
                info "Now perform: rm -f ${so} && ln -s ${so1_name} ${so}"
                rm -f ${so} && ln -s ${so1_name} ${so}
            fi
        fi
    done
}

function tbb_relink() {
    TBB_LIBDIR="/usr/local/lib"
    for so in ${TBB_LIBDIR}/libtbb*.so*; do
        soname="$(basename ${so})"
        IFS='.' read -ra arr <<< "${soname}"
        IFS=' ' # restore IFS
        num=${#arr[@]}
        if [[ ${num} != 4 ]]; then # Keep only libtbb.so.12.1
            rm -f ${so} || true
        fi
    done

    for so in ${TBB_LIBDIR}/libtbb*.so*; do
        soname="$(basename ${so})"
        IFS='.' read -ra arr <<< "${soname}"
        IFS=' ' # restore IFS
        core="${arr[0]}.so"
        ln -s ${soname} ${TBB_LIBDIR}/${core}.${arr[2]}
        ln -s ${core}.${arr[2]} ${TBB_LIBDIR}/${core}
    done
}

mkl_relink
tbb_relink
ldconfig

ok "Successfully installed mkl from PYPI"
