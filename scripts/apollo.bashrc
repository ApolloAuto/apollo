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

APOLLO_ROOT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export APOLLO_ROOT_DIR

: ${VERBOSE:=yes}
export TAB="    " # 4 spaces

BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

function info() {
  (>&2 echo -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() {
  (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() {
  (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() {
  (>&2 echo -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}

function print_delim() {
  echo "=============================================="
}

function get_now() {
    date +%s
}

function time_elapsed_s() {
    local start="${1:-$(get_now)}"
    local end="$(get_now)"
    echo "$end - $start" | bc -l
}

function success() {
  print_delim
  ok "$1"
  print_delim
}

function fail() {
  print_delim
  error "$1"
  print_delim
  exit 1
}

function file_ext() {
  local __ext="${1##*.}"
  if [ "${__ext}" == "$1" ]; then
    __ext=""
  fi
  echo "${__ext}"
}

function c_family_ext() {
  local __ext
  __ext="$(file_ext $1)"
  for ext in "h" "hh" "hxx" "hpp" "cxx" "cc" "cpp" "cu"; do
    if [ "${ext}" == "${__ext}" ]; then
      return 0
    fi
  done
  return 1
}

function find_c_cpp_srcs() {
  find "$@" -type f -name "*.h"   \
                 -o -name "*.c"   \
                 -o -name "*.hpp" \
                 -o -name "*.cpp" \
                 -o -name "*.hh"  \
                 -o -name "*.cc"  \
                 -o -name "*.hxx" \
                 -o -name "*.cxx" \
                 -o -name "*.cu"
}

## Prevent multiple entries of my_bin_path in PATH
function add_to_path() {
  if [ -z "$1" ]; then
    return
  fi
  local my_bin_path="$1"
  if [ -n "${PATH##*${my_bin_path}}" ] && [ -n "${PATH##*${my_bin_path}:*}" ]; then
    export PATH=$PATH:${my_bin_path}
  fi
}

# Exits the script if the command fails.
function run() {
  if [ "${VERBOSE}" = yes ]; then
    echo "${@}"
    "${@}" || exit $?
  else
    local errfile="${APOLLO_ROOT_DIR}/.errors.log"
    echo "${@}" >"${errfile}"
    if ! "${@}" >>"${errfile}" 2>&1; then
      local exitcode=$?
      cat "${errfile}" 1>&2
      exit $exitcode
    fi
  fi
}

#commit_id=$(git log -1 --pretty=%H)
function git_sha1() {
  if [ -x "$(which git 2>/dev/null)" ] && \
     [ -d "${APOLLO_ROOT_DIR}/.git" ]; then
    git rev-parse --short HEAD 2>/dev/null || true
  fi
}

function git_date() {
  if [ -x "$(which git 2>/dev/null)" ] && \
     [ -d "${APOLLO_ROOT_DIR}/.git" ]; then
    git log -1 --pretty=%ai | cut -d " " -f 1 || true
  fi
}

function git_branch() {
  if [ -x "$(which git 2>/dev/null)" ] && \
     [ -d "${APOLLO_ROOT_DIR}/.git" ]; then
    git rev-parse --abbrev-ref HEAD
  else
    echo "@non-git"
  fi
}

function read_one_char_from_stdin() {
    local answer
    read -r -n1 answer
    # Bash 4.x+: ${answer,,} to lowercase, ${answer^^} to uppercase
    echo "${answer}" | tr '[:upper:]' '[:lower:]'
}
