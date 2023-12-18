#! /usr/bin/env bash
set -e

INSTALL_TARGETS=()
SUBDIR_TARGETS=()
CPU_INSTALL_TARGETS=()
CPU_SUBDIR_TARGETS=()

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

export OPT_APOLLO="$(dirname "${APOLLO_SYSROOT_DIR}")"
export PREFIX_DIR="${PREFIX_DIR:=${APOLLO_DISTRIBUTION_HOME}}"

LIST_ONLY=0
RESOLVE_DEPS=0
PRE_CLEAN=0
BAZEL_OPTS=" -c opt --copt=-mavx2 --host_copt=-mavx2 --jobs=$(nproc) --local_ram_resources=HOST_RAM*0.5 --cxxopt=-fPIC"
SHORTHAND_TARGETS=
CMDLINE_OPTIONS=
INSTALL_OPTIONS=
USE_GPU=-1
LEGACY_RELEASE=0
USER_INPUT_PREFIX=

function _usage() {
  info "Usage: $0 <module>(Can be empty) [Options]"
  info "Options:"
  info "${TAB} -p, --prefix <DIR> Use absolute path <DIR> as install prefix instead of '/apollo/output'"
  info "${TAB} -l, --list         Print the list of installed files; don't install anything"
  info "${TAB} -c, --clean        Ensure clean install by removing prefix dir if exist before installing"
  info "${TAB} -r, --resolve      Also resolve APT packages on which this release build depends"
  info "${TAB} --legacy           Legacy way to release apollo output"
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
  local remained_args=
  while [[ $# -gt 0 ]]; do
    local opt="$1"
    shift
    case "${opt}" in
      -p | --prefix)
        _check_arg_for_opt "${opt}" "$1"
        prefix_dir="$1"
        shift
        USER_INPUT_PREFIX="${prefix_dir}"
        ;;
      -l | --list)
        LIST_ONLY=1
        ;;
      --legacy)
        LEGACY_RELEASE=1
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
        remained_args="${remained_args} ${opt}"
        ;;
    esac
  done
  if [[ "${RESOLVE_DEPS}" -gt 0 && "${LIST_ONLY}" -gt 0 ]]; then
    error "'-l,--list' and '-r,--resolve' cannot be used together"
    _usage
    exit 1
  fi
  if [[ "${prefix_dir}" == /* ]]; then
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
    # INSTALL_OPTIONS=" --gpu ${INSTALL_OPTIONS}"
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
  ldd $1 | awk '/=>/ {print $3}' | sort -u |
    grep -Ev "^(${OPT_APOLLO}|/usr/local|${PREFIX_DIR})/"
}
export -f retrieve_so_deps

function generate_solibs() {
  listing="$1"
  find ${PREFIX_DIR}/bin ${APOLLO_SYSROOT_DIR}/bin -executable -type f \
    -exec bash -c 'retrieve_so_deps "$0"' {} \
    \; >> ${listing}
  find ${PREFIX_DIR}/lib ${PREFIX_DIR}/cyber ${PREFIX_DIR}/modules \
    -name "*.so" -exec bash -c 'retrieve_so_deps "$0"' {} \
    \; >> ${listing}

  SYSLIB_DIRS=(
    /usr/local/fast-rtps/lib
    /usr/local/libtorch_cpu/lib
    /usr/local/libtorch_gpu/lib
    ${APOLLO_SYSROOT_DIR}/lib
  )

  for libdir in ${SYSLIB_DIRS[@]}; do
    find ${libdir} \( -type f -or -type l \) -name "*.so" \
      -exec bash -c 'retrieve_so_deps "$0"' {} \
      \; >> ${listing}
  done
  find /usr/local/qt5/ -name "*.so" -exec bash -c 'retrieve_so_deps "$0"' {} \
    \; >> ${listing}

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
  if ! msg="$(dpkg -S ${solib} 2> /dev/null)"; then
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
  local install_target=()
  local install_src_target=()
  # local search_install_targets=`bazel query 'attr(name, install, //modules/... except //modules/perception2/...)' 2>/dev/null`
  # local serach_install_3rd_targets=`bazel query 'attr(name, install, //third_party/rtklib/...)' 2>/dev/null`

  # search_install_targets+=(${search_install_3rd_targets[@]})

  # for instance in ${search_install_targets[*]}; do
  # instance_list=(${instance//:/ })
  # if [[ ${instance_list[1]} == install ]]; then
  #     install_target[${#install_target[*]}]=${instance}
  # elif [[ ${instance_list[1]} == install_src ]]; then
  #     install_src_target[${#install_src_target[*]}]=${instance}
  # fi
  # done

  for d in ${SUBDIR_TARGETS[*]}; do
    sub_dirs=$(ls ${d})
    for dir in ${sub_dirs[*]}; do
      if [[ ${dir} == BUILD ]]; then
        continue
      elif [[ ${dir} == README.md ]]; then
        continue
      elif [[ ${dir} == production ]]; then
        continue
      elif [[ ${dir} == camera_overlap_filter ]]; then
        continue
      elif [[ ${dir} == hdmap_based_proposal ]]; then
        continue
      elif [[ ${dir} == launch ]]; then
        continue
      fi
      INSTALL_TARGETS[${#INSTALL_TARGETS[*]}]="//${d}/${dir}:install"
    done
  done

  for d in ${CPU_SUBDIR_TARGETS[*]}; do
    sub_dirs=$(ls ${d})
    for dir in ${sub_dirs[*]}; do
      if [[ ${dir} == BUILD ]]; then
        continue
      elif [[ ${dir} == README.md ]]; then
        continue
      elif [[ ${dir} == production ]]; then
        continue
      elif [[ ${dir} == camera_overlap_filter ]]; then
        continue
      elif [[ ${dir} == hdmap_based_proposal ]]; then
        continue
      elif [[ ${dir} == launch ]]; then
        continue
      fi
      CPU_INSTALL_TARGETS[${#CPU_INSTALL_TARGETS[*]}]="//${d}/${dir}:install"
    done
  done

  if [[ "${LEGACY_RELEASE}" -gt 0 ]]; then
    bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} //:install \
      -- ${install_opts} ${INSTALL_OPTIONS} ${PREFIX_DIR}
  else

    # for target in ${INSTALL_TARGETS[*]}; do
    #   bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} ${target}_src \
    #     -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"
    # done

    # for target in ${INSTALL_TARGETS[*]}; do
    #   bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} ${target} \
    #     -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"
    # done

    # CMDLINE_OPTIONS="--config=cpu "

    # for target in ${CPU_INSTALL_TARGETS[*]}; do
    #   bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} ${target}_src \
    #     -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"
    # done

    # for target in ${CPU_INSTALL_TARGETS[*]}; do
    #   bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} ${target} \
    #     -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"
    # done

    bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} //:deprecated_install \
      -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"

    # install files copy from source code.
    bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} //:deprecated_install_src \
      -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"
    CMDLINE_OPTIONS="--config=gpu "
    bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} //:deprecated_install \
      -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"

    # install files copy from source code.
    bazel run ${BAZEL_OPTS} ${CMDLINE_OPTIONS} //:deprecated_install_src \
      -- ${install_opts} ${INSTALL_OPTIONS} "${PREFIX_DIR}"

  fi

}

function export_python_path() {
  if [ $(grep -c /opt/apollo/neo/packages/python-support/local ~/.bashrc) -ne 0 ]; then
    echo '\nexport PYTHONPATH=/opt/apollo/neo/packages/python-support/local:$PYTHONPATH' >> ~/.bashrc
  fi
}

function main() {
  parse_cmdline_args "$@"

  # force to use gpu build to release package
  USE_GPU=1
  determine_cpu_or_gpu_build

  local install_opts=
  if [[ "${LIST_ONLY}" -gt 0 ]]; then
    install_opts="${install_opts} --list"
  fi
  if [[ "${PRE_CLEAN}" -gt 0 ]]; then
    install_opts="${install_opts} --pre_clean"
  fi

  if [[ "${LEGACY_RELEASE}" -gt 0 ]]; then
    PREFIX_DIR=${prefix_dir:="${PWD}/output"}
    install_opts="${install_opts} --legacy"
  fi

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

  if [[ "${LEGACY_RELEASE}" -gt 0 ]]; then
    generate_py_packages
    resolve_directory_path
  fi
}

main "$@"
