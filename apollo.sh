#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"
SUPPORTED_ARCHS=" x86_64 aarch64 "
APOLLO_VERSION="@non-git"
APOLLO_ENV=""

USE_ESD_CAN=false
: ${STAGE:=dev}

AVAILABLE_COMMANDS="config build build_dbg build_opt build_cpu build_gpu build_opt_gpu test coverage lint \
                    buildify check build_fe build_teleop build_prof doc clean format usage -h --help"

function check_architecture_support() {
  if [[ "${SUPPORTED_ARCHS}" != *" ${ARCH} "* ]]; then
    error "Unsupported CPU arch: ${ARCH}. Currently, Apollo only" \
      "supports running on the following CPU archs:"
    error "${TAB}${SUPPORTED_ARCHS}"
    exit 1
  fi
}

function check_platform_support() {
  local platform="$(uname -s)"
  if [[ "${platform}" != "Linux" ]]; then
    error "Unsupported platform: ${platform}."
    error "${TAB}Apollo is expected to run on Linux systems (E.g., Debian/Ubuntu)."
    exit 1
  fi
}

function check_minimal_memory_requirement() {
  local minimal_mem_gb="2.0"
  local actual_mem_gb="$(free -m | awk '/Mem:/ {printf("%0.2f", $2 / 1024.0)}')"
  if (($(echo "$actual_mem_gb < $minimal_mem_gb" | bc -l))); then
    warning "System memory [${actual_mem_gb}G] is lower than the minimum required" \
      "[${minimal_mem_gb}G]. Apollo build could fail."
  fi
}

function determine_esdcan_use() {
  local esdcan_dir="${APOLLO_ROOT_DIR}/third_party/can_card_library/esd_can"
  local use_esd=false
  if [[ "${ARCH}" == "x86_64" ]] &&
    [[ -f "${esdcan_dir}/include/ntcan.h" ]] &&
    [[ -f "${esdcan_dir}/lib/libntcan.so.4" ]]; then
    use_esd=true
  fi
  USE_ESD_CAN="${use_esd}"
}

function check_apollo_version() {
  local branch="$(git_branch)"
  if [ "${branch}" == "${APOLLO_VERSION}" ]; then
    return
  fi
  local sha1="$(git_sha1)"
  local stamp="$(git_date)"
  APOLLO_VERSION="${branch}-${stamp}-${sha1}"
}

function apollo_env_setup() {
  check_apollo_version

  check_architecture_support
  check_platform_support
  check_minimal_memory_requirement
  setup_gpu_support
  determine_esdcan_use

  APOLLO_ENV="STAGE=${STAGE}"
  APOLLO_ENV="${APOLLO_ENV} USE_ESD_CAN=${USE_ESD_CAN}"
  # Add more here ...

  info "${BLUE}Apollo Environment Settings:${NO_COLOR}"
  info "${TAB}APOLLO_ROOT_DIR:   ${GREEN}${APOLLO_ROOT_DIR}${NO_COLOR}"
  info "${TAB}APOLLO_CACHE_DIR:  ${GREEN}${APOLLO_CACHE_DIR}${NO_COLOR}"
  info "${TAB}APOLLO_IN_DOCKER:  ${GREEN}${APOLLO_IN_DOCKER}${NO_COLOR}"
  info "${TAB}APOLLO_VERSION:    ${GREEN}${APOLLO_VERSION}${NO_COLOR}"
  if "${APOLLO_IN_DOCKER}"; then
    info "${TAB}DOCKER_IMG:        ${GREEN}${DOCKER_IMG##*:}${NO_COLOR}"
  fi
  info "${TAB}APOLLO_ENV:        ${GREEN}${APOLLO_ENV}${NO_COLOR}"
  info "${TAB}USE_GPU_HOST:      ${GREEN}${USE_GPU_HOST}${NO_COLOR}"
  info "${TAB}USE_GPU_TARGET:    ${GREEN}${USE_GPU_TARGET}${NO_COLOR}"
  info "${TAB}GPU_PLATFORM:      ${GREEN}${GPU_PLATFORM}${NO_COLOR}"

  if [[ -z "${APOLLO_BAZEL_DIST_DIR}" ]]; then
    source "${TOP_DIR}/cyber/setup.bash"
  fi
  if [[ ! -d "${APOLLO_BAZEL_DIST_DIR}" ]]; then
    mkdir -p "${APOLLO_BAZEL_DIST_DIR}"
  fi

  env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_config.sh" --noninteractive
}

#TODO(all): Update node modules
function build_dreamview_frontend() {
  pushd "${APOLLO_ROOT_DIR}/modules/dreamview/frontend" > /dev/null
  yarn build
  popd > /dev/null
}

function build_test_and_lint() {
  env ${APOLLO_ENV} bash "${build_sh}"
  env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test
  env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh" --cpp
  success "Build and Test and Lint finished."
}

function _usage() {
  echo -e "\n${RED}Usage${NO_COLOR}:
    .${BOLD}/apollo.sh${NO_COLOR} [OPTION]"
  echo -e "\n${RED}Options${NO_COLOR}:
    ${BLUE}config [options]${NO_COLOR}: config bazel build environment either non-interactively (default) or interactively.
    ${BLUE}build [module]${NO_COLOR}: run build for cyber (<module> = cyber) or modules/<module>.  If <module> unspecified, build all.
    ${BLUE}build_dbg [module]${NO_COLOR}: run debug build.
    ${BLUE}build_opt [module]${NO_COLOR}: run optimized build.
    ${BLUE}build_cpu [module]${NO_COLOR}: build in CPU mode. Equivalent to 'bazel build --config=cpu'
    ${BLUE}build_gpu [module]${NO_COLOR}: run build in GPU mode. Equivalent to 'bazel build --config=gpu'
    ${BLUE}build_nvidia [module]${NO_COLOR}: run build in GPU mode for NVIDIA GPU target. Equivalent to 'bazel build --config=nvidia'
    ${BLUE}build_amd [module]${NO_COLOR}: run build in GPU mode for AMD GPU target. Equivalent to 'bazel build --config=amd'
    ${BLUE}build_opt_gpu [module]${NO_COLOR}: optimized build in GPU mode. Equivalent to 'bazel build --config=opt --config=gpu'
    ${BLUE}build_opt_nvidia [module]${NO_COLOR}: optimized build in GPU mode for NVIDIA GPU target. Equivalent to 'bazel build --config=opt --config=nvidia'
    ${BLUE}build_opt_amd [module]${NO_COLOR}: optimized build in GPU mode for AMD GPU target. Equivalent to 'bazel build --config=opt --config=amd'
    ${BLUE}build_pkg [module]${NO_COLOR}: build apollo on package-management way
    ${BLUE}build_pkg_dbg [module]${NO_COLOR}: build apollo on package-management way
    ${BLUE}build_pkg_opt [module]${NO_COLOR}: build apollo on package-management way
    ${BLUE}build_pkg_opt_gpu [module]${NO_COLOR}: build apollo on package-management way
    ${BLUE}test [module]${NO_COLOR}: run unittest for cyber (module='cyber') or modules/<module>. If unspecified, test all.
    ${BLUE}coverage [module]${NO_COLOR}: run coverage test for cyber (module='cyber') or modules/<module>. If unspecified, coverage all.
    ${BLUE}lint${NO_COLOR}: run code style check
    ${BLUE}buildify${NO_COLOR}: run 'buildifier' to fix style of bazel files.
    ${BLUE}check${NO_COLOR}: run build, test and lint on all modules. Recommmened before checking in new code.
    ${BLUE}build_fe${NO_COLOR}: compile frontend JS code for Dreamview. Requires all node_modules pre-installed.
    ${BLUE}build_teleop${NO_COLOR}: run build with teleop enabled.
    ${BLUE}build_prof [module]${NO_COLOR}: build with perf profiling support. Not implemented yet.
    ${BLUE}install_dv_plugins ${NO_COLOR}: install Dreamview plugins package.
    ${BLUE}doc${NO_COLOR}: generate doxygen document
    ${BLUE}release${NO_COLOR}: build Apollo binary releases
    ${BLUE}clean${NO_COLOR}: cleanup bazel output and log/coredump files
    ${BLUE}format${NO_COLOR}: format C++/Python/Bazel/Shell files
    ${BLUE}usage${NO_COLOR}: show this message and exit
    "
}

function _check_command() {
  local name="${BASH_SOURCE[0]}"
  local commands="$(echo ${AVAILABLE_COMMANDS} | xargs)"
  local help_msg="Run './apollo.sh --help' for usage."
  local cmd="$@"

  python scripts/command_checker.py --name "${name}" --command "${cmd}" --available "${commands}" --helpmsg "${help_msg}"
}

function check_config_cpu() {
  if [[ $* == *--config?cpu* ]]; then
    export USE_GPU_TARGET="0"
  fi
}

function main() {
  if [ "$#" -eq 0 ]; then
    _usage
    exit 0
  fi

  check_config_cpu "$@"
  apollo_env_setup

  local build_sh="${APOLLO_ROOT_DIR}/scripts/apollo_build.sh"
  local build_pkg_sh="${APOLLO_ROOT_DIR}/scripts/apollo_build_pkg.sh"
  local pkg_sh="${APOLLO_ROOT_DIR}/scripts/apollo_build_package.sh"
  local test_sh="${APOLLO_ROOT_DIR}/scripts/apollo_test.sh"
  local coverage_sh="${APOLLO_ROOT_DIR}/scripts/apollo_coverage.sh"
  local ci_sh="${APOLLO_ROOT_DIR}/scripts/apollo_ci.sh"

  local cmd="$1"
  shift
  case "${cmd}" in
    config)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_config.sh" "$@"
      ;;
    build)
      env ${APOLLO_ENV} bash "${build_sh}" "$@"
      ;;
    build_opt)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt "$@"
      ;;
    build_dbg)
      env ${APOLLO_ENV} bash "${build_sh}" --config=dbg "$@"
      ;;
    build_cpu)
      env ${APOLLO_ENV} bash "${build_sh}" --config=cpu "$@"
      ;;
    build_gpu)
      env ${APOLLO_ENV} bash "${build_sh}" --config=gpu "$@"
      ;;
    build_nvidia)
      env ${APOLLO_ENV} bash "${build_sh}" --config=nvidia "$@"
      ;;
    build_amd)
      env ${APOLLO_ENV} bash "${build_sh}" --config=amd "$@"
      ;;
    build_opt_gpu)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=gpu "$@"
      ;;
    build_opt_gpu_pnc)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=gpu "cyber planning prediction control routing dreamview external_command tools common_msgs"
      ;;
    build_pnc)
      env ${APOLLO_ENV} bash "${build_sh}" --config=dbg --config=gpu "cyber planning prediction control routing dreamview external_command tools common_msgs"
      ;;
    build_pkg)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" "$@"
      ;;
    build_pkg_opt)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" --config=opt "$@"
      ;;
    build_pkg_opt_gpu)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" --config=opt --config=gpu "$@"
      ;;
    build_pkg_dbg)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" --config=dbg "$@"
      ;;
    build_opt_nvidia)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=nvidia "$@"
      ;;
    build_opt_amd)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=amd "$@"
      ;;
    build_prof)
      env ${APOLLO_ENV} bash "${build_sh}" --config=prof "$@"
      ;;
    build_teleop)
      env ${APOLLO_ENV} bash "${build_sh}" --config=teleop "$@"
      ;;
    build_fe)
      build_dreamview_frontend
      ;;
    test)
      env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test "$@"
      ;;
    coverage)
      env ${APOLLO_ENV} bash "${coverage_sh}" "$@"
      ;;
    cibuild)
      env ${APOLLO_ENV} bash "${ci_sh}" "build"
      ;;
    citest)
      env ${APOLLO_ENV} bash "${ci_sh}" "test"
      ;;
    cilint)
      env ${APOLLO_ENV} bash "${ci_sh}" "lint"
      ;;
    check)
      build_test_and_lint
      ;;
    buildify)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_buildify.sh"
      ;;
    lint)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh" "$@"
      ;;
    clean)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_clean.sh" "$@"
      ;;
    release)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_release.sh" "$@"
      ;;
    install_dv_plugins)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/install_dv_plugins.sh"
      ;;
    doc)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_docs.sh" "$@"
      ;;
    format)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_format.sh" "$@"
      ;;
    usage)
      _usage
      ;;
    -h | --help)
      _usage
      ;;
    *)
      _check_command "${cmd}"
      ;;
  esac
}

main "$@"
