# usage: source cyber_tools_auto_complete.bash

function _cyber_launch_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'cyber_launch')
    COMPREPLY=( $(compgen -W "start stop" -- ${word}) )
    ;;
  'start')
    compopt -o nospace
    local files=`ls *.launch 2>/dev/null`
    COMPREPLY=( $(compgen -W "$files" -- ${word}) )
    ;;
  'stop')
    compopt -o nospace
    local files=`ls *.launch 2>/dev/null`
    COMPREPLY=( $(compgen -W "$files" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _cyber_recorder_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'cyber_recorder')
    COMPREPLY=( $(compgen -W "play info record split recover" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _cyber_channel_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'cyber_channel')
    COMPREPLY=( $(compgen -W "echo list info hz bw type" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _cyber_node_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'cyber_node')
    COMPREPLY=( $(compgen -W "list info" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _cyber_service_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'cyber_service')
    COMPREPLY=( $(compgen -W "list info" -- ${word}) )
    ;;
  *)
    ;;
  esac
}
complete -F _cyber_launch_complete -o default cyber_launch
complete -F _cyber_recorder_complete -o default cyber_recorder
complete -F _cyber_channel_complete -o default cyber_channel
complete -F _cyber_node_complete -o default cyber_node
complete -F _cyber_service_complete -o default cyber_service
