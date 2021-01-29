#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

export OPT_APOLLO="$(dirname "${APOLLO_SYSROOT_DIR}")"
export PREFIX_DIR=/apollo/output

LIST_ONLY=0
RESOLVE_DEPS=0
PRE_CLEAN=0
BAZEL_OPTS="--config=opt --config=gpu"

function _usage() {
    info "Usage: $0 [Options]"
    info "Options:"
    info "${TAB} -p, --prefix <DIR> Use absolute path <DIR> as install prefix instead of '/apollo/output'"
    info "${TAB} -l, --list         Print the list of installed files; don't install anything"
    info "${TAB} -c, --clean        Ensure clean install by removing prefix dir if exist before installing"
    info "${TAB} -r, --resolve      Also resolve APT packages on which this release build depends"
    info "${TAB} -h, --help         Show this message and exit"
}

function _check_arg_for_opt() {
    local opt="$1"
    local optarg="$2"
    if [[ -z "${optarg}" || "${optarg}" =~ ^-.* ]]; then
        error "Missing argument for ${opt}. Exiting..."
        exit 2
    fi
}

function parse_cmdline_args() {
    local prefix_dir=
    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case "${opt}" in
            -p | --prefix)
                _check_arg_for_opt "${opt}" "$1"
                prefix_dir="$1"; shift
                ;;
            -l | --list)
                LIST_ONLY=1
                ;;
            -r | --resolve)
                RESOLVE_DEPS=1
                ;;
            -c | --clean)
                PRE_CLEAN=1
                ;;
            -h | --help)
                _usage
                exit 0
                ;;
            *)
                error "Unknown option: ${opt}"
                _usage
                exit 1
                ;;
        esac
    done
    if [[ "${RESOLVE_DEPS}" -gt 0 && "${LIST_ONLY}" -gt 0 ]]; then
        error "'-l,--list' and '-r,--resolve' cannot be used together"
        _usage
        exit 1
    fi
    if [[ "${prefix_dir}" = /* ]]; then
        PREFIX_DIR="${prefix_dir}"
    elif [[ -n "${prefix_dir}" ]]; then
        echo "Absolute prefix dir expected, got '${prefix_dir}'"
        exit 1
    fi
}

function retrieve_so_deps() {
    ldd $1 | awk '/=>/ {print $3}' | sort -u | \
        grep -Ev "^(${OPT_APOLLO}|/usr/local|${PREFIX_DIR})/"
}
export -f retrieve_so_deps

function generate_solibs() {
    listing="$1"
    find ${PREFIX_DIR}/bin ${APOLLO_SYSROOT_DIR}/bin -executable -type f  \
        -exec bash -c 'retrieve_so_deps "$0"' {}  \
            >> ${listing} \;
    find ${PREFIX_DIR}/lib ${PREFIX_DIR}/cyber ${PREFIX_DIR}/modules \
        -name "*.so" -exec bash -c 'retrieve_so_deps "$0"' {}  \
            >> ${listing} \;

    SYSLIB_DIRS=(
        /usr/local/fast-rtps/lib
        /usr/local/libtorch_cpu/lib
        /usr/local/libtorch_gpu/lib
        ${APOLLO_SYSROOT_DIR}/lib
    )

    for libdir in ${SYSLIB_DIRS[@]}; do
        find ${libdir} -name "*.so" \
            -exec bash -c 'retrieve_so_deps "$0"' {} \
                >> ${listing} \;
    done
    cat ${listing} | sort -u
}

function solib_locate() {
    solib="$1"
    dest="$2"
    # https://superuser.com/questions/363444
    # /how-do-i-get-the-output-and-exit-value-of-a-subshell-when-using-bash-e
    if ! msg="$(dpkg -S ${solib} 2>/dev/null)" ; then
        echo "Warning: ${solib} doesn't seem to belong to any APT package."
    else
        result="$(echo "${msg}" | awk -F ': ' '{print $1}')"
        echo "${result}" >> ${dest}
    fi
}

PKGS_TXT="${PREFIX_DIR}/pkgs.txt"
function generate_apt_pkgs() {
    sudo apt-get -y update
    listing="$(mktemp /tmp/syslibs.XXXXXX)"
    pkgs="$(mktemp /tmp/pkgs.XXXXXX)"
    for solib in $(generate_solibs "${listing}"); do
        solib_locate "${solib}" "${pkgs}"
    done
    sort -u ${pkgs} > ${PKGS_TXT}
    rm -f ${listing} ${pkgs}
}

function main() {
    parse_cmdline_args "$@"

    local install_opts=
    if [[ "${LIST_ONLY}" -gt 0 ]]; then
        install_opts="${install_opts} --list"
    fi
    if [[ "${PRE_CLEAN}" -gt 0 ]]; then
        install_opts="${install_opts} --pre_clean"
    fi
    bazel run ${BAZEL_OPTS} //:install \
        -- ${install_opts} "${PREFIX_DIR}"

    if [[ "${LIST_ONLY}" -gt 0 ]]; then
        return
    fi

    info "Resolve directory path for Apollo binary distribution..."
    find "${PREFIX_DIR}" -name "*.dag" -exec \
        sed -i 's@/apollo/bazel-bin@/apollo@g' {} \;
    ok "Done."
    if [[ "${RESOLVE_DEPS}" -gt 0 ]]; then
        info "Resolve runtime library dependencies and generate APT packages list..."
        generate_apt_pkgs
        ok "Done. Packages list has been writen to ${PKGS_TXT}"
    fi
}

main "$@"
