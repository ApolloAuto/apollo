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

NPM_DOCS="https://docs.npmjs.com/downloading-and-installing-node-js-and-npm"

function check_if_tools_installed() {
  if [ ! -x "$(command -v npm)" ]; then
    error "Please install npm first. Refer to ${NPM_DOCS} for help."
    exit 1
  fi

  if ! npm list -g | grep -q prettier; then
    error "'prettier' not found. Please install it manually by running:"
    error "${TAB}sudo npm install -g --save-dev --save-exact prettier"
    exit 1
  fi
}

function prettier_run() {
  npx prettier --write "$@"
}

function format_markdown_by_prettier() {
  for mypath in "$@"; do
    if [ -d "${mypath}" ]; then
      local srcs="$(find_prettier_srcs ${mypath})"
      if [[ -n "${srcs}" ]]; then
        prettier_run ${srcs}
      fi
      ok "Done formatting markdown/json/yaml files under ${mypath}."
    elif [ -f "${mypath}" ]; then
      if prettier_ext "${mypath}"; then
        prettier_run "${mypath}"
      else
        warning "Only regular markdown/json/yaml files will be formatted. Ignored ${mypath}"
      fi
    else
      warning "Special/Symlink file won't be formatted. Ignored ${mypath}"
    fi
  done
}

function main() {
  if [ "$#" -eq 0 ]; then
    warning "Usage: $0 <path/to/markdown/dir/or/file>"
    exit 1
  fi

  check_if_tools_installed
  format_markdown_by_prettier "$@"
}

main "$@"
