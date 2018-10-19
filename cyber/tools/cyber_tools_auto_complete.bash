# source cyber_tools_auto_complete.bash

function _cyber_launch_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  local pattern="*.launch"
  case $cmd in
  'cyber_launch')
    COMPREPLY=( $(compgen -W "start" -- ${word}) )
    ;;
  'start')
    compopt -o nospace
    local files=`cd ${CYBER_PATH}/launch/ 2>/dev/null && ls -l *.launch |awk '{print $NF}' 2>/dev/null`
    COMPREPLY=( $(compgen -W "$files" -- ${word}) )
    #COMPREPLY=( $(compgen -d -f -G "$pattern" -- ${word}) )
    ;;
  '*')
    ;;
  esac
}

function _cyber_recorder_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  local subcmd="play info record split recover"
  local pattern="*.record"
  case $cmd in
  'cyber_recorder')
    COMPREPLY=( $(compgen -W "$subcmd" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _cyber_channel_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  local subcmd="list echo info hz bw"
  case $cmd in
  'cyber_channel')
    COMPREPLY=( $(compgen -W "$subcmd" -- ${word}) )
    ;;
  '*')
    ;;
  esac
}

function _cyber_param_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  local subcmd="list get set remove load dump"
  case $cmd in
  'cyber_param')
    COMPREPLY=( $(compgen -W "$subcmd" -- ${word}) )
    ;;
  '*')
    ;;
  esac
}

function _cyber_service_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  local subcmd="list info"
  case $cmd in
  'cyber_service')
    COMPREPLY=( $(compgen -W "$subcmd" -- ${word}) )
    ;;
  '*')
    ;;
  esac
}

function _cyber_node_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  local subcmd="list info"
  case $cmd in
  'cyber_node')
    COMPREPLY=( $(compgen -W "$subcmd" -- ${word}) )
    ;;
  '*')
    ;;
  esac
}

complete -F _cyber_launch_complete -o default cyber_launch
complete -F _cyber_recorder_complete -o default cyber_recorder
complete -F _cyber_channel_complete cyber_channel
complete -F _cyber_param_complete  cyber_param
complete -F _cyber_service_complete  cyber_service
complete -F _cyber_node_complete  cyber_node
