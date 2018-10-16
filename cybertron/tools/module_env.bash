#!/usr/bin/env bash

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
    local files=`cd ${MODULE_PATH}/launch/ 2>/dev/null && ls -l *.launch |awk '{print $NF}' 2>/dev/null`
    COMPREPLY=( $(compgen -W "$files" -- ${word}) )
    #COMPREPLY=( $(compgen -d -f -G "$pattern" -- ${word}) )
    ;;
  '*')
    ;;
  esac
}

export MODULE_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo "MODULE_PATH is" ${MODULE_PATH}
library_path=${MODULE_PATH}/lib/
binary_path=${MODULE_PATH}/bin/

export LD_LIBRARY_PATH=${library_path}:$LD_LIBRARY_PATH
export PATH=${binary_path}:$PATH

complete -F _cyber_launch_complete -o default cyber_launch
