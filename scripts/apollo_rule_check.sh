#! /usr/bin/env bash

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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

BAZEL_PACKAGE=()
TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"

readdir() {
  if [[ -f "${1}/BUILD" ]]; then
    BAZEL_PACKAGE[${#BAZEL_PACKAGE[*]}]="${1}"
  fi
  for file in `ls -r $1`
    do
      [[ ${file} == bazel-* ]] && continue
      [[ ${file} == third_party ]] && continue
      if [ -d $1/$file ];then
        cd $1/$file
        readdir $1"/"$file
        cd -
      fi
    done
}

function main {
  pushd ${TOP_DIR} 
  
  if [[ -z `which buildozer` ]]; then
    arch=`uname -m`
    url="https://apollo-system.cdn.bcebos.com/archive/9.0/buildozer-linux-"
    [[ $arch -eq "x86_64" ]] && url="${url}amd64"
    [[ $arch -eq "aarch64" ]] && url="${url}arm64" 
    wget ${url} -O ~/buildozer && sudo chmod 777 ~/buildozer
    sudo ln -snf ~/buildozer /usr/bin/buildozer
  fi
  readdir ${TOP_DIR}
  for subpackage in ${BAZEL_PACKAGE[*]}; do
    cc_library_ret=`buildozer 'print name' "${subpackage}":%cc_library`
    [[ $? != 0 ]] && return $?
    cc_binary_ret=`buildozer 'print name' "${subpackage}":%cc_binary`
    [[ $? != 0 ]] && return $?
    cc_test_ret=`buildozer 'print name' "${subpackage}":%cc_test`
    [[ $? != 0 ]] && return $?
    if [[ ${cc_library_ret} != '' || ${cc_binary_ret} != '' || ${cc_test_ret} != '' ]]; 
    then
      error "bazel native cc rule detected in ${subpackage}/BUILD"
      popd
      return -1
    fi
  done
  info "bazel native cc rule test passed"
  popd
  return 0
}

main