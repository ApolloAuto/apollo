# usage: source auto_complete.bash

COMMANDS="start start_cpu start_gpu enter remove stopall bootstrap build install init profile create list update setup_host usage -h --help"

function _complete_func() {
    COMPREPLY=()
    local cur="${COMP_WORDS[COMP_CWORD]}"
    local cmds="$(echo ${COMMANDS} | xargs)"

    COMPREPLY=($(compgen -W "${cmds}" -- ${cur}))

}

complete -F _complete_func -o default aem
