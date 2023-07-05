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
source "${TOP_DIR}/scripts/apollo.bashrc"
source "${TOP_DIR}/scripts/apollo_base.sh"

BAZEL_OUT="${TOP_DIR}/bazel-out" # $(bazel info output_path)
COVERAGE_HTML="${TOP_DIR}/.cache/coverage"
COVERAGE_DAT="${BAZEL_OUT}/_coverage/_coverage_report.dat"

# Note(storypku): branch coverage seems not work when running bazel coverage
# GENHTML_OPTIONS="--rc genhtml_branch_coverage=1 --highlight --legend"

function main() {
  parse_cmdline_arguments "$@"
  run_bazel "Coverage" "$@"
  genhtml "${COVERAGE_DAT}" --output-directory "${COVERAGE_HTML}"
  success "Done bazel coverage for ${SHORTHAND_TARGETS:-Apollo}"
  info "Coverage report was generated under ${COVERAGE_HTML}"
}

main "$@"
