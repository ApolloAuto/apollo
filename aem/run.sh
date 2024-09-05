#!/bin/bash
#
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
export SCRIPT_DIR=$(
  cd $(dirname $0)
  pwd
)
export SCRIPT_REAL_DIR=$(dirname $(realpath $0))
[[ -f ${SCRIPT_REAL_DIR}/env.sh ]] && source ${SCRIPT_REAL_DIR}/env.sh
[[ -f ${SCRIPT_REAL_DIR}/funcs.sh ]] && source ${SCRIPT_REAL_DIR}/funcs.sh

export AEM_INITED=1

export CSI_SGR_PARAM_RESET=0
export CSI_SGR_PARAM_BOLD=1
export CSI_SGR_PARAM_DIM=2
export CSI_SGR_PARAM_ITALIC=3
export CSI_SGR_PARAM_UNDERLINE=4
export CSI_SGR_PARAM_BLINK=5
export CSI_SGR_PARAM_REVERSE=7
export CSI_SGR_PARAM_HIDDEN=8
export CSI_SGR_PARAM_STRIKE=9
export CSI_SGR_PARAM_NORMAL=22
export CSI_SGR_PARAM_UNDERLINE_OFF=24
export CSI_SGR_PARAM_BLINK_OFF=25
export CSI_SGR_PARAM_REVERSE_OFF=27
export CSI_SGR_PARAM_FG_BLACK=30
export CSI_SGR_PARAM_FG_RED=31
export CSI_SGR_PARAM_FG_GREEN=32
export CSI_SGR_PARAM_FG_YELLOW=33
export CSI_SGR_PARAM_FG_BLUE=34
export CSI_SGR_PARAM_FG_MAGENTA=35
export CSI_SGR_PARAM_FG_CYAN=36
export CSI_SGR_PARAM_FG_WHITE=37
export CSI_SGR_PARAM_BG_BLACK=40
export CSI_SGR_PARAM_BG_RED=41
export CSI_SGR_PARAM_BG_GREEN=42
export CSI_SGR_PARAM_BG_YELLOW=43
export CSI_SGR_PARAM_BG_BLUE=44
export CSI_SGR_PARAM_BG_MAGENTA=45
export CSI_SGR_PARAM_BG_CYAN=46
export CSI_SGR_PARAM_BG_WHITE=47

join_by() {
  local IFS="$1"
  shift
  echo "$*"
}
export -f join_by

colorize() {
  local msg="$1"
  shift
  ctrls=("$@")
  local ctrl_str="$(join_by ';' "${ctrls[@]}")"
  echo -e "\033[${ctrl_str}m${msg}\033[0m"
}
export -f colorize

log() {
  local level="$1"
  shift
  local msg="$@"
  echo -e "[${level}] $(date +"%Y-%m-%d %H:%M:%S") ${msg}" >&2
}
export -f log

debug() {
  level="DEBUG"
  level_params=(
    "${CSI_SGR_PARAM_ITALIC}"
    "${CSI_SGR_PARAM_FG_MAGENTA}"
  )
  msg_params=(
    "${CSI_SGR_PARAM_ITALIC}"
    "${CSI_SGR_PARAM_FG_MAGENTA}"
  )
  log "$(colorize "${level}" "${level_params[@]}")" \
    "$(colorize "$*" "${msg_params[@]}")"
}
export -f debug

info() {
  level="INFO "
  level_params=(
    "${CSI_SGR_PARAM_BOLD}"
    "${CSI_SGR_PARAM_FG_BLUE}"
  )
  msg_params=(
    "${CSI_SGR_PARAM_BOLD}"
    "${CSI_SGR_PARAM_FG_BLUE}"
  )
  log "$(colorize "${level}" "${level_params[@]}")" \
    "$(colorize "$*" "${msg_params[@]}")"
}
export -f info

warn() {
  level="WARN "
  level_params=(
    "${CSI_SGR_PARAM_BOLD}"
    "${CSI_SGR_PARAM_FG_YELLOW}"
  )
  msg_params=(
    "${CSI_SGR_PARAM_BOLD}"
    "${CSI_SGR_PARAM_FG_YELLOW}"
  )
  log "$(colorize "${level}" "${level_params[@]}")" \
    "$(colorize "$*" "${msg_params[@]}")"
}
export -f warn

error() {
  level="ERROR"
  level_params=(
    "${CSI_SGR_PARAM_FG_RED}"
    "${CSI_SGR_PARAM_NORMAL}"
  )
  msg_params=(
    "${CSI_SGR_PARAM_BOLD}"
    "${CSI_SGR_PARAM_FG_RED}"
  )
  log "$(colorize "${level}" "${level_params[@]}")" \
    "$(colorize "$*" "${msg_params[@]}")"
}
export -f error

fatal() {
  level="FATAL"
  level_params=(
    "${CSI_SGR_PARAM_BOLD}"
    "${CSI_SGR_PARAM_FG_RED}"
  )
  msg_params=(
    "${CSI_SGR_PARAM_BOLD}"
    "${CSI_SGR_PARAM_BLINK}"
    "${CSI_SGR_PARAM_FG_RED}"
    "${CSI_SGR_PARAM_BG_YELLOW}"
  )
  log "$(colorize "${level}" "${level_params[@]}")" \
    "$(colorize "$*" "${msg_params[@]}")"
}
export -f fatal

die() {
  fatal "$@"
  exit 1
}
export -f die

###############################################################################
# Breif:
#   Exit if procedure check failure
# Arguments:
#   $1: command to be run
#   $@: arguemts to be passed to command
# Returns:
#   0: run command succeed
#   ?: run command failed with exit code $?
###############################################################################
run() {
  local cmd=$1
  shift
  ${cmd} "$@"
  local ret=$?
  [[ "$ret" != "0" ]] &&
    error "command error $ret: ${cmd} $@, exiting" &&
    return $ret
  return 0
}
export -f run

usage() {
  echo "Usage:
    $0 <command> [...args]
Examples:
    "
}

run_subcommand() {
  :
}

aem_usage() {
  echo "Usage:
    $0 <subcommand> [...args]
Subcommands:
  create|start      Create and start a new environment
  remove            Remove and stop an existing environment
  list              List all available environments
  enter             Enter an environment
  exec              Execute a command in an environment
  help              Show this help message
  bootstrap         Launch dreamview and monitor
  profile           Manage profiles
"
}

load_sys_commands() {
  if [[ ! -d "${SCRIPT_REAL_DIR}/commands.d" ]]; then
    return 0
  fi
  for file in ${SCRIPT_REAL_DIR}/commands.d/*.sh; do
    source ${file}
  done
}

load_sys_shared_plugins() {
  if [[ ! -d "${SCRIPT_REAL_DIR}/plugins.d" ]]; then
    return 0
  fi
  for file in ${AEM_SYS_SHARE}/plugins.d/*.sh; do
    source ${file}
  done
}

load_user_plugins() {
  if [[ ! -d "$HOME/.aem/plugins.d" ]]; then
    return 0
  fi
  for file in $HOME/.aem/plugins.d/*.sh; do
    source ${file}
  done
}

aem() {
  if [[ "$#" == 0 ]]; then
    error "no subcommand specified"
    aem_usage
    return 1
  fi

  # load_sys_commands
  # load_sys_shared_plugins
  # load_user_plugins

  local cmd="$1"
  if [[ "${cmd}" == "help" ]] || [[ "${cmd}" == "-h" ]] || [[ "${cmd}" == "--help" ]]; then
    aem_usage
    return 0
  fi

  cmd_path="$(command -v "aem-${cmd}" || true)"
  if [[ -z "${cmd_path}" ]]; then
    if [[ -x "${SCRIPT_DIR}/aem-${cmd}" ]]; then
      cmd_path="${SCRIPT_DIR}/aem-${cmd}"
    elif [[ -x "${SCRIPT_REAL_DIR}/aem-${cmd}" ]]; then
      cmd_path="${SCRIPT_REAL_DIR}/aem-${cmd}"
    else
      error "no such command \`${cmd}'"
      return 0
    fi
  fi
  shift 1
  exec "${cmd_path}" "$@"

  # local cmd_func="aem_${cmd}"
  # local cmd_usage="aem_${cmd}_usage"

  # if [[ ! "$(type -t "${cmd_func}")" == "function" ]]; then
  #   err "subcommand not found: ${cmd}"
  #   aem_usage
  #   return 1
  # fi
  # shift
  # run "${cmd_func}" "$@"
  # ret="$?"
  # if [[ "${ret}" != "0" ]]; then
  #   if [[ "$(type -t "aem_${cmd}_usage")" == "function" ]]; then
  #     run "aem_${cmd}_usage"
  #   fi
  #   return "${ret}"
  # fi
}
export -f aem

main() {
  if [[ (-L "$0") && ("$(type -t $(basename $0))" == "function") ]]; then
    # symlink alias, use filename as command
    cmd="$(basename $0)"
  else
    cmd=$1
    if [[ "$#" > 0 ]]; then
      shift
    fi
  fi
  if [[ "${cmd}" == "" ]]; then
    cmd="usage"
  fi
  run ${cmd} "$@"
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  main "$@"
fi
