#! /usr/bin/env bash

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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
## 加载 Apollo 基础环境
source "${TOP_DIR}/scripts/apollo_base.sh"

function main() {
  # 调用 site_restore 函数（该函数定义在 apollo_base.sh），可能用于 恢复某些 Apollo 依赖项或环境设置
  site_restore
  # 解析传入的命令行参数
  parse_cmdline_arguments "$@"
  # 运行 Bazel 进行构建
  run_bazel "Build"
  if [ -z "${SHORTHAND_TARGETS}" ]; then
    SHORTHAND_TARGETS="apollo"
  fi
  success "Done building ${SHORTHAND_TARGETS}. Enjoy!"
}
# 运行 main 函数，传入所有命令行参数
main "$@"
