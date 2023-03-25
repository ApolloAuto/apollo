#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

export OPT_APOLLO="$(dirname "${APOLLO_SYSROOT_DIR}")"
export PREFIX_DIR=/opt/apollo/neo/packages/

LIST_ONLY=0
RESOLVE_DEPS=0
PRE_CLEAN=0
BAZEL_OPTS=" -c opt --copt=-mavx2 --host_copt=-mavx2 --jobs=$(nproc) --local_ram_resources=HOST_RAM*0.5"
SHORTHAND_TARGETS=
CMDLINE_OPTIONS=
INSTALL_OPTIONS=
USE_GPU=-1

function _usage() {
    info "Usage: $0 <module>(Can be empty) [Options]"
    info "Options:"
    info "${TAB} -p, --prefix <DIR> Use absolute path <DIR> as install prefix instead of '/apollo/output'"
    info "${TAB} -l, --list         Print the list of installed files; don't install anything"
    info "${TAB} -c, --clean        Ensure clean install by removing prefix dir if exist before installing"
    info "${TAB} -r, --resolve      Also resolve APT packages on which this release build depends"
    info "${TAB} -h, --help         Show this message and exit"
    info "${TAB} --gpu              Running GPU build"
    info "${TAB} --cpu              Running CPU build"
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
    local remained_args=
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
            --cpu)
                USE_GPU=0
                ;;
            --gpu)
                USE_GPU=1
                ;;
            *)
                remained_args="${remained_args} ${opt}"
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
    SHORTHAND_TARGETS="${remained_args}"
}

function determine_cpu_or_gpu_build() {
  if [ "${USE_GPU}" -lt 0 ]; then
    # USE_GPU unset, defaults to USE_GPU_TARGET
    USE_GPU="${USE_GPU_TARGET}"
  elif [ "${USE_GPU}" -gt "${USE_GPU_TARGET}" ]; then
    warning "USE_GPU=${USE_GPU} without GPU can't compile. Exiting ..."
    exit 1
  fi

  if [ "${USE_GPU}" -eq 1 ]; then
    CMDLINE_OPTIONS="--config=gpu ${CMDLINE_OPTIONS}"
    INSTALL_OPTIONS=" --gpu ${INSTALL_OPTIONS}"

    ok "Running GPU build."
  else
    CMDLINE_OPTIONS="--config=cpu ${CMDLINE_OPTIONS}"
    ok "Running CPU build."
  fi
}

function determine_release_targets() {
    local targets_all
    if [[ "$#" -eq 0 ]]; then
        targets_all="//:install"
        echo "${targets_all}"
        return
    fi

    for component in $@; do
        local release_targets
        if [ "${component}" = "cyber" ]; then
            release_targets="//cyber:install"
        elif [[ -d "${APOLLO_ROOT_DIR}/modules/${component}" ]]; then
            release_targets="//modules/${component}:install"
        elif [ "${component}" = "scripts" ]; then
            release_targets="//docker/scripts:install union //scripts:install"
        else
            error "Unknown option: ${component}"
            _usage
            exit 1
        fi
        if [ -z "${targets_all}" ]; then
          targets_all="${release_targets}"
        else
          targets_all="${targets_all} union ${release_targets}"
        fi
    done
    echo "${targets_all}"
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
        find ${libdir} \( -type f -or -type l \) -name "*.so" \
            -exec bash -c 'retrieve_so_deps "$0"' {} \
                >> ${listing} \;
    done
    find /usr/local/qt5/ -name "*.so" -exec bash -c 'retrieve_so_deps "$0"' {}  \
            >> ${listing} \;

    cat ${listing} | sort -u
}

function solib_locate() {
    solib="$1"
    if [[ ${solib} != "/"* || ! -e ${solib} ]]; then
        return
    fi
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

PKGS_TXT="${PREFIX_DIR}/syspkgs.txt"
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

function generate_py_packages() {
    pushd ${PREFIX_DIR} > /dev/null
    touch __init__.py
    for dir in $(find cyber modules -type d); do
      touch $dir/__init__.py
    done
    popd > /dev/null
}

function resolve_directory_path() {
    pushd ${PREFIX_DIR} > /dev/null
    info "Resolve directory path for Apollo binary distribution..."
    mkdir -p bazel-bin
    ln -s ../cyber bazel-bin/cyber
    ln -s ../modules bazel-bin/modules
    ok "Done."
    popd > /dev/null
}

function run_install() {
    local install_targets
    install_targets="$(determine_release_targets ${SHORTHAND_TARGETS})"
    bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} ${install_targets} \
        -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"

    # install files copy from source code.
    bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} //:install_src \
        -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"
}

function export_python_path() {
    if [ `grep -c /opt/apollo/neo/packages/python-support/local ~/.bashrc` -ne 0 ]; then
        echo '\nexport PYTHONPATH=/opt/apollo/neo/packages/python-support/local:$PYTHONPATH' >> ~/.bashrc
    fi
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

    determine_cpu_or_gpu_build

    run_install

    if [[ "${LIST_ONLY}" -gt 0 ]]; then
        return
    fi

    DIRS=("${PREFIX_DIR}/data/log" "${PREFIX_DIR}/data/bag" "${PREFIX_DIR}/data/core")

    for dir in ${DIRS[@]}; do
        if [[ ! -d "${dir}" ]]; then
          mkdir -p "${dir}"
        fi
    done

    if [[ "${RESOLVE_DEPS}" -gt 0 ]]; then
        info "Resolve runtime library dependencies and generate APT packages list..."
        generate_apt_pkgs
        ok "Done. Packages list has been writen to ${PKGS_TXT}"
    fi

    export_python_path

    # generate_py_packages
    # resolve_directory_path
}

main "$@"
