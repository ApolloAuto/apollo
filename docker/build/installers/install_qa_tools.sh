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

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

TARGET_ARCH="$(uname -m)"

## NOTE:
## buildifier/buildozer was moved into install_bazel.sh.

apt_get_update_and_install \
    lcov
#    cppcheck
#    valgrind

# libgoogle-perftools4  # gperftools
# PROFILER_SO="/usr/lib/${TARGET_ARCH}-linux-gnu/libprofiler.so"
# if [ ! -e "${PROFILER_SO}" ]; then
#    # libgoogle-perftools4: /usr/lib/x86_64-linux-gnu/libprofiler.so.0
#    ln -s "${PROFILER_SO}.0" "${PROFILER_SO}"
# fi

bash ${CURR_DIR}/install_shellcheck.sh
bash ${CURR_DIR}/install_gperftools.sh
bash ${CURR_DIR}/install_benchmark.sh
# TechDoc generation
bash ${CURR_DIR}/install_doxygen.sh

# sphinx ?

## Pylint
pip3_install pycodestyle \
    pyflakes \
    flake8 \
    yapf \
    autopep8
# pylint

# shfmt
bash ${CURR_DIR}/install_shfmt.sh

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
