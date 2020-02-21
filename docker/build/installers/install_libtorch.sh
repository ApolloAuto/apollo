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

cd "$(dirname "${BASH_SOURCE[0]}")"

pip3 install --no-cache-dir PyYAML typing

git clone --recursive --single-branch --branch apollo --depth 1 https://github.com/ApolloAuto/pytorch.git

pushd pytorch
  export USE_CUDA=0
  python3 setup.py install
  mkdir -p /usr/local/apollo/libtorch
  cp -r build/lib.linux-x86_64-3.6/torch/lib /usr/local/apollo/libtorch/
  cp -r build/lib.linux-x86_64-3.6/torch/include /usr/local/apollo/libtorch/

  python3 setup.py clean

  export USE_CUDA=1
  export TORCH_CUDA_ARCH_LIST="3.5;5.0;5.2;6.1;7.0;7.5"

  python3 setup.py install
  mkdir -p /usr/local/apollo/libtorch_gpu
  cp -r build/lib.linux-x86_64-3.6/torch/lib /usr/local/apollo/libtorch_gpu/
  cp -r build/lib.linux-x86_64-3.6/torch/include /usr/local/apollo/libtorch_gpu/

popd
rm -fr pytorch
