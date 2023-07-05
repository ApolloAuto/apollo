# usage: source apollo_auto_complete.bash

COMMANDS="config build build_dbg build_opt build_cpu build_gpu build_opt_gpu test coverage lint \
          buildify check build_fe build_teleop build_prof doc clean format usage -h --help"

MODULES="$(find /apollo/modules/* -maxdepth 0 -type d -printf "%f ")"
MODULES="cyber ${MODULES}"

function _complete_apollo_func() {
  COMPREPLY=()
  local cur="${COMP_WORDS[COMP_CWORD]}"
  local prev="${COMP_WORDS[COMP_CWORD - 1]}"

  local cmds="$(echo ${COMMANDS} | xargs)"
  local modules="$(echo ${MODULES} | xargs)"

  if [ "${COMP_CWORD}" -eq 1 ]; then
    COMPREPLY=($(compgen -W "${cmds}" -- ${cur}))
  elif [ "${COMP_CWORD}" -eq 2 ]; then
    case "${prev}" in
      build | build_dbg | build_opt | build_cpu | build_gpu | test | coverage)
        COMPREPLY=($(compgen -W "${modules}" -- ${cur}))
        ;;
      config)
        COMPREPLY=($(compgen -W "--interactive --noninteractive --help" -- ${cur}))
        ;;
      clean)
        COMPREPLY=($(compgen -W "--bazel --core --log --expunge --all --help" -- ${cur}))
        ;;
      lint)
        COMPREPLY=($(compgen -W "--py --sh --cpp --all --help" -- ${cur}))
        ;;
      format)
        COMPREPLY=($(compgen -W "--python --bazel --cpp --shell --markdown --all --help" -- ${cur}))
        ;;
    esac
  fi
}

complete -F _complete_apollo_func -o default apollo.sh
