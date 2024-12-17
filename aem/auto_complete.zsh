#compdef aem

_arguments \
    '-h --help'{-h,--help}'[display help information]' \
    '1: :->command' \
    '*:: :->args'

case $state in
    command)
        _values 'aem' \
            'create[create a dev container]' \
            'start[start a dev container]' \
            'start_gpu[start a dev container with gpu devices]' \
            'enter[enter a dev container]' \
            'remove[remove a dev container]' \
            'list[list all dev containers]' \
            'bootstrap[run dreamview and monitor module]' \
            'build[build packages in workspace]' \
            'install[install source code of package to workspace]' \
            'init[initialize workspace]' \
            'profile[profiles management]'

        ;;
    args)
        case $line[1] in
            create)
                _arguments -s \
                    {-t,--tag}'[tag of the docker image]' \
                    {-n,--name}'[name of the env]' \
                    {-l,--local}'[use local images]' \
                    {-m,--mount}'[mount directory]' \
                    '(--user)--user[custom user]' \
                    '(--uid)--uid[custom uid]' \
                    '(--group)--group[custom group]' \
                    '(--gid)--gid[custom gid]' 

                ;;
            start|start_gpu)
                _arguments -s \
                    {-t,--tag}'[tag of the docker image]' \
                    {-n,--name}'[name of the env]' \
                    {-f,--force}'[force mode]' \
                    {-l,--local}'[use local images]' \
                    {-m,--mount}'[mount directory]' \
                    '(--user)--user[custom user]' \
                    '(--uid)--uid[custom uid]' \
                    '(--group)--group[custom group]' \
                    '(--gid)--gid[custom gid]'

                ;;
            enter)
                _aem_enter
                ;;
            remove)
                _aem_remove
                ;;
        esac

        ;;
esac

_aem_enter() {
    local line state
    _arguments \
        {-n,--name}'[name of the env]:name:->names' \
        '(--user)--user[custom user]'

    case "${state}" in
        names)
            local envs_home=${APOLLO_ENVS_ROOT:-$HOME/.apollo/aem/envs}
            local container_prefix=${DEV_CONTAINER_PREFIX:-apollo_neo_dev_}
            local containers=($(ls -1 ${envs_home}))
            if [[ ${#containers[@]} -eq 0 ]]; then
                _message 'no containers'
                return
            fi
            _values 'containers' ${containers[@]//${container_prefix}/}
            ;;
    esac
}

_aem_remove() {
    local line state
    _arguments \
        {-n,--name}'[name of the env]:name:->names'

    case "${state}" in
        names)
            local envs_home=${APOLLO_ENVS_ROOT:-$HOME/.apollo/aem/envs}
            local container_prefix=${DEV_CONTAINER_PREFIX:-apollo_neo_dev_}
            local containers=($(ls -1 ${envs_home}))
            if [[ ${#containers[@]} -eq 0 ]]; then
                _message 'no containers to remove'
                return
            fi
            _values 'containers' ${containers[@]//${container_prefix}/}
            ;;
    esac
}
