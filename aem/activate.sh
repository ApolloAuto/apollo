#!/bin/bash
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

# Flags
#set -u
#set -e
#set -x

if [[ "${BASH_SOURCE-}" == "$0" ]]; then
  echo "this script should be sourced, e.g. source $0" >&2
  exit 1
fi

APOLLO_PATH_ENV_NAMES=(
  'APOLLO_MODEL_PATH'
  'APOLLO_LIB_PATH'
  'APOLLO_DAG_PATH'
  'APOLLO_FLAG_PATH'
  'APOLLO_CONF_PATH'
  'APOLLO_LAUNCH_PATH'
  'APOLLO_PLUGIN_INDEX_PATH'
  'APOLLO_PLUGIN_LIB_PATH'
  'APOLLO_PLUGIN_DESCRIPTION_PATH'
)

function pathremove() {
  local IFS=':'
  local NEWPATH
  local DIR
  local PATHVARIABLE=${2:-PATH}
  for DIR in ${!PATHVARIABLE}; do
    if [ "$DIR" != "$1" ]; then
      NEWPATH=${NEWPATH:+$NEWPATH:}$DIR
    fi
  done
  export $PATHVARIABLE="$NEWPATH"
}

function pathprepend() {
  pathremove $1 $2
  local PATHVARIABLE=${2:-PATH}
  export $PATHVARIABLE="$1${!PATHVARIABLE:+:${!PATHVARIABLE}}"
}

function pathappend() {
  pathremove $1 $2
  local PATHVARIABLE=${2:-PATH}
  export $PATHVARIABLE="${!PATHVARIABLE:+${!PATHVARIABLE}:}$1"
}

function generate_ld_library_path() {
  cat ${APOLLO_ENV_ROOT}/etc/ld.so.conf.d/apollo.conf |
    grep -v -E '^\s*$' |
    grep -v -E '^\s*#.*$' |
    tr '\n' ':'
}

deactivate() {
  for name in "${APOLLO_PATH_ENV_NAMES[@]}"; do
    unset $name
  done

  restore_env TENSORRT_VERSION
  restore_env LD_LIBRARY_PATH
  restore_env PYTHONPATH
  restore_env PATH

}

backup_env() {
  name="${1}"
  backup_name="__ORIG__${name}"
  if [[ -n "${!name}" ]]; then
    export "${backup_name}"="${!name}"
  fi
}

restore_env() {
  name="${1}"
  backup_name="__ORIG__${name}"
  if [[ -n "${!backup_name}" ]]; then
    export "${name}"="${!backup_name}"
    unset "${backup_name}"
  fi
}

deactivate

# export APOLLO_ENV_WORKROOT="${APOLLO_ENV_ROOT}"

# paths
export APOLLO_MODEL_PATH="${APOLLO_ENV_ROOT}/apollo/modules/perception/data/models"
export APOLLO_LIB_PATH="${APOLLO_ENV_ROOT}/lib"
export APOLLO_DAG_PATH="${APOLLO_ENV_ROOT}/apollo"
export APOLLO_FLAG_PATH="${APOLLO_ENV_ROOT}/apollo"
export APOLLO_CONF_PATH="${APOLLO_ENV_ROOT}/apollo"
export APOLLO_LAUNCH_PATH="${APOLLO_ENV_ROOT}/apollo"
export APOLLO_RUNTIME_PATH="${APOLLO_ENV_ROOT}/apollo"
# TODO: set `/apollo/apollo/neo` as prefix
export APOLLO_PLUGIN_INDEX_PATH="${APOLLO_ENV_ROOT}/opt/apollo/neo/share/cyber_plugin_index"
export APOLLO_PLUGIN_LIB_PATH="${APOLLO_ENV_ROOT}/opt/apollo/neo/lib"
export APOLLO_PLUGIN_DESCRIPTION_PATH="${APOLLO_ENV_ROOT}/opt/apollo/neo/share"

# runtime variables
export AEM_HOST_VIRTUALENV=1
export APOLLO_DISTRIBUTION_HOME="${APOLLO_DISTRIBUTION_HOME:-${APOLLO_ENV_ROOT}/opt/apollo/neo}"
export APOLLO_SYSROOT_DIR="${APOLLO_SYSROOT_DIR:-/opt/apollo/sysroot}"
export APOLLO_CACHE_DIR="${APOLLO_CACHE_DIR:-./.cache}"
export APOLLO_BAZEL_DIST_DIR="${APOLLO_BAZEL_DIST_DIR:-${APOLLO_CACHE_DIR}/distdir}"
export APOLLO_ROOT_DIR="${APOLLO_ROOT_DIR:-${APOLLO_ENV_ROOT}/apollo}"
export APOLLO_PATH="${APOLLO_PATH:-${APOLLO_ENV_ROOT}/opt/apollo/neo}"
export GLOG_log_dir="${GLOG_log_dir:-${APOLLO_ENV_ROOT}/apollo/data/log}"
export CYBER_PATH="${CYBER_PATH:-${APOLLO_ROOT_DIR}/cyber}"
export CYBER_IP="${CYBER_IP:-127.0.0.1}"
export CYBER_DOMAIN_ID="${CYBER_DOMAIN_ID:-80}"
export APOLLO_CONFIG_HOME="${APOLLO_CONFIG_HOME:-${HOME}/.apollo}"

# TODO: detect automatically
backup_env TENSORRT_VERSION
export TENSORRT_VERSION=7.2.1

backup_env LD_LIBRARY_PATH
# pathprepend "${APOLLO_LIB_PATH}" LD_LIBRARY_PATH
# pathprepend "${APOLLO_PLUGIN_LIB_PATH}" LD_LIBRARY_PATH
# # TODO: move to APOLLO_ENV_ROOT
# pathprepend "/usr/local/fast-rtps/lib" LD_LIBRARY_PATH
# pathprepend "/usr/local/libtorch_gpu/lib" LD_LIBRARY_PATH

backup_env PYTHONPATH
pathprepend "${APOLLO_ENV_ROOT}/opt/apollo/neo/python" PYTHONPATH

backup_env PATH
pathprepend "${APOLLO_SYSROOT_DIR}/bin" PATH
pathprepend "${APOLLO_ENV_ROOT}/bin" PATH
pathprepend "${APOLLO_ENV_ROOT}/opt/apollo/neo/bin" PATH

PS1="\[\e[31m\][\[\e[m\]\[\e[32m\]\u\[\e[m\]\[\e[33m\]@\[\e[m\]\[\e[35m\]\h\[\e[m\]:\[\e[36m\]\w\[\e[m\]\[\e[31m\]]\[\e[m\]\[\e[1;32m\]\$\[\e[m\] "

alias ls='ls --color=auto'
alias buildtool='_abt() {
    command buildtool "$@"
    export LD_LIBRARY_PATH="$(generate_ld_library_path)"
};_abt'

# ensure directorys exists
mkdir -p "${APOLLO_CONFIG_HOME}"
mkdir -p "${CYBER_PATH}"
mkdir -p "${APOLLO_ENV_ROOT}/apollo/data/log"
mkdir -p "${APOLLO_ENV_ROOT}/apollo/data/core"
mkdir -p "${APOLLO_ENV_ROOT}/apollo/data/bag"
