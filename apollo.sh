## 指定使用 bash 作为解释器执行该脚本
#! /usr/bin/env bash
# 在脚本执行过程中，任何一个命令如果返回非零值（即执行失败），脚本就会立即退出，以防止错误继续传播
set -e

# 计算当前脚本所在的绝对路径，即 Apollo 项目根目录
# BASH_SOURCE[0] 代表当前脚本的路径
# dirname "${BASH_SOURCE[0]}" 获取脚本所在目录
# cd "$(dirname "${BASH_SOURCE[0]}")" 进入该目录
# pwd -P 获取当前目录的绝对路径（防止软链接影响）
TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# 加载 Apollo 的 bashrc 配置文件，初始化环境变量
source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"  # 获取当前 CPU 架构（例如 x86_64 或 aarch64）
SUPPORTED_ARCHS=" x86_64 aarch64 "   #  定义 Apollo 仅支持的 CPU 架构
APOLLO_VERSION="@non-git"   # 默认 Apollo 版本号，稍后会动态更新
APOLLO_ENV=""  # Apollo 运行环境变量

USE_ESD_CAN=false  # 是否使用 ESD CAN 设备，默认 false
: ${STAGE:=dev}   #  如果 STAGE 没有被定义，则默认设置为 dev（开发环境）
# AVAILABLE_COMMANDS：定义 Apollo 支持的命令，例如 build（编译）、test（运行测试）、clean（清理编译缓存）等
# build：编译 Apollo
# test：运行单元测试
# clean：清理构建产物
# lint：代码风格检查
# format：格式化代码
# help：显示帮助信息
AVAILABLE_COMMANDS="config build build_dbg build_opt build_cpu build_gpu build_opt_gpu test coverage lint \
                    buildify check build_fe build_teleop build_prof doc clean format usage -h --help"

# check_architecture_support()：检查当前系统的 CPU 架构是否被 Apollo 支持
# if [[ "${SUPPORTED_ARCHS}" != *" ${ARCH} "* ]]; then ... exit 1：如果 CPU 架构不在支持列表内，输出错误信息并终止脚本
function check_architecture_support() {
  if [[ "${SUPPORTED_ARCHS}" != *" ${ARCH} "* ]]; then
    error "Unsupported CPU arch: ${ARCH}. Currently, Apollo only" \
      "supports running on the following CPU archs:"
    error "${TAB}${SUPPORTED_ARCHS}"
    exit 1
  fi
}
# check_platform_support()：检查操作系统是否为 Linux，否则终止脚本
# uname -s 获取 当前操作系统名称
function check_platform_support() {
  local platform="$(uname -s)"
  if [[ "${platform}" != "Linux" ]]; then
    error "Unsupported platform: ${platform}."
    error "${TAB}Apollo is expected to run on Linux systems (E.g., Debian/Ubuntu)."
    exit 1
  fi
}
# check_minimal_memory_requirement()：检查系统内存是否小于 2GB，并发出警告
# free -m：获取系统内存（单位：MB）
# awk '/Mem:/ {printf("%0.2f", $2 / 1024.0)}'：获取总内存值，并转换为 GB
# bc -l：使用 浮点运算 判断 actual_mem_gb < minimal_mem_gb，如果 内存不足 2GB，则 警告 用户
function check_minimal_memory_requirement() {
  local minimal_mem_gb="2.0"
  local actual_mem_gb="$(free -m | awk '/Mem:/ {printf("%0.2f", $2 / 1024.0)}')"
  if (($(echo "$actual_mem_gb < $minimal_mem_gb" | bc -l))); then
    warning "System memory [${actual_mem_gb}G] is lower than the minimum required" \
      "[${minimal_mem_gb}G]. Apollo build could fail."
  fi
}
# determine_esdcan_use()：检测 esd_can 驱动文件是否存在，并决定是否启用 ESD CAN 设备
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
# 检查 Apollo 版本
function check_apollo_version() {
  local branch="$(git_branch)"
  if [ "${branch}" == "${APOLLO_VERSION}" ]; then
    return
  fi
  local sha1="$(git_sha1)"
  local stamp="$(git_date)"
  APOLLO_VERSION="${branch}-${stamp}-${sha1}"
}
# apollo_env_setup()：依次调用 check_apollo_version()、check_architecture_support() 等函数，确保 Apollo 运行环境正确
function apollo_env_setup() {
  check_apollo_version  #检查 Apollo 版本

  check_architecture_support  # 检查 CPU 架构
  check_platform_support    # 检查操作系统
  check_minimal_memory_requirement   # 检查内存
  setup_gpu_support  # 设置 GPU 支持
  determine_esdcan_use  #  检查 CAN 总线设备

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
# build_dreamview_frontend()：进入 dreamview/frontend 目录，运行 yarn build 构建前端代码
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

# main()：主函数，根据用户输入的命令执行对应的 Apollo 任务
# ./apollo.sh build -> 运行 apollo_build.sh 进行编译
# ./apollo.sh test -> 运行 apollo_test.sh 进行单元测试
# ./apollo.sh clean -> 清理编译产物
function main() {
  if [ "$#" -eq 0 ]; then
    _usage
    exit 0
  fi
# 先运行 apollo_env_setup 初始化环境
  check_config_cpu "$@"
  apollo_env_setup

  local build_sh="${APOLLO_ROOT_DIR}/scripts/apollo_build.sh"
  local build_pkg_sh="${APOLLO_ROOT_DIR}/scripts/apollo_build_pkg.sh"
  local pkg_sh="${APOLLO_ROOT_DIR}/scripts/apollo_build_package.sh"
  local test_sh="${APOLLO_ROOT_DIR}/scripts/apollo_test.sh"
  local coverage_sh="${APOLLO_ROOT_DIR}/scripts/apollo_coverage.sh"
  local ci_sh="${APOLLO_ROOT_DIR}/scripts/apollo_ci.sh"

  local cmd="$1"  #获取第一个命令行参数，并存入变量 cmd
  shift # 左移参数列表，删除第一个参数，使 $2 变为 $1，$3 变为 $2
  case "${cmd}" in
# config 命令调用 apollo_config.sh 脚本进行环境配置。
# env ${APOLLO_ENV}：设置 APOLLO_ENV 环境变量。
# "$@"：传递所有剩余的参数
    config)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_config.sh" "$@"
      ;;
    build)
      env ${APOLLO_ENV} bash "${build_sh}" "$@"
      ;;
    # 优化编译（--config=opt）
    build_opt)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt "$@"
      ;;
    # 调试模式编译（--config=dbg）
    build_dbg)
      env ${APOLLO_ENV} bash "${build_sh}" --config=dbg "$@"
      ;;
    # CPU 版本编译（--config=cpu）
    build_cpu)
      env ${APOLLO_ENV} bash "${build_sh}" --config=cpu "$@"
      ;;
    # GPU 版本编译（--config=gpu）
    build_gpu)
      env ${APOLLO_ENV} bash "${build_sh}" --config=gpu "$@"
      ;;
    # 针对 Nvidia GPU 进行编译（--config=nvidia）
    build_nvidia)
      env ${APOLLO_ENV} bash "${build_sh}" --config=nvidia "$@"
      ;;
    # 针对 AMD GPU 进行编译（--config=amd）
    build_amd)
      env ${APOLLO_ENV} bash "${build_sh}" --config=amd "$@"
      ;;
    # 优化模式 + GPU 编译
    build_opt_gpu)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=gpu "$@"
      ;;
    # 优化 + GPU，仅编译 cyber、planning、prediction 等关键模块
    build_opt_gpu_pnc)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=gpu "cyber planning prediction control routing dreamview external_command tools common_msgs"
      ;;
    # 调试模式 + GPU，仅编译 PNC 相关模块
    build_pnc)
      env ${APOLLO_ENV} bash "${build_sh}" --config=dbg --config=gpu "cyber planning prediction control routing dreamview external_command tools common_msgs"
      ;;
    # 编译指定的单个软件包
    build_pkg)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" "$@"
      ;;
    # 优化模式编译单个包
    build_pkg_opt)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" --config=opt "$@"
      ;;
    # 优化 + GPU 编译单个包
    build_pkg_opt_gpu)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" --config=opt --config=gpu "$@"
      ;;
    # 调试模式编译单个包
    build_pkg_dbg)
      env ${APOLLO_ENV} bash "${build_pkg_sh}" --config=dbg "$@"
      ;;
    build_opt_nvidia)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=nvidia "$@"
      ;;
    build_opt_amd)
      env ${APOLLO_ENV} bash "${build_sh}" --config=opt --config=amd "$@"
      ;;
    # 性能分析模式编译（--config=prof）
    build_prof)
      env ${APOLLO_ENV} bash "${build_sh}" --config=prof "$@"
      ;;
    # 远程驾驶模式编译（--config=teleop）
    build_teleop)
      env ${APOLLO_ENV} bash "${build_sh}" --config=teleop "$@"
      ;;
    # 编译 DreamView 前端
    build_fe)
      build_dreamview_frontend
      ;;
    # 运行单元测试（--config=unit_test）
    test)
      env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test "$@"
      ;;
    # 运行代码覆盖率测试
    coverage)
      env ${APOLLO_ENV} bash "${coverage_sh}" "$@"
      ;;
    # CI 编译
    cibuild)
      env ${APOLLO_ENV} bash "${ci_sh}" "build"
      ;;
    # CI 测试
    citest)
      env ${APOLLO_ENV} bash "${ci_sh}" "test"
      ;;
    # CI 代码风格检查（Lint）
    cilint)
      env ${APOLLO_ENV} bash "${ci_sh}" "lint"
      ;;
    # 编译、测试和 Lint 一起执行
    check)
      build_test_and_lint
      ;;
    # 格式化 Bazel 相关文件
    buildify)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_buildify.sh"
      ;;
    # 运行代码风格检查
    lint)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh" "$@"
      ;;
    # 清理编译生成的文件
    clean)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_clean.sh" "$@"
      ;;
    # 执行发布操作
    release)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_release.sh" "$@"
      ;;
    # 安装 DreamView 插件
    install_dv_plugins)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/install_dv_plugins.sh"
      ;;
    # 生成 Apollo 文档
    doc)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_docs.sh" "$@"
      ;;
    # 格式化代码
    format)
      env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_format.sh" "$@"
      ;;
    # usage 或 -h / --help：显示帮助信息
    usage)
      _usage
      ;;
    -h | --help)
      _usage
      ;;
    # 如果 cmd 不匹配任何已定义的命令，则执行 _check_command 进行检查
    *)
      _check_command "${cmd}"
      ;;
  esac
}

main "$@"
