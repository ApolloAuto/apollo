#! /usr/bin/env bash

set -e

APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${APOLLO_ROOT_DIR}/scripts/apollo.bashrc"
TEST_CMD="bazel test --distdir=${APOLLO_CACHE_DIR}"

function check_pyflakes() {

    local pyflakes_cmd="$(command -v pyflakes)"
    if [ -z "${pyflakes_cmd}" ]; then  
      sudo -H pip3 install pyflakes 
    fi

}

function check_shellcheck() {

    local shellcheck_cmd="$(command -v shellcheck)"
    if [ -z "${shellcheck_cmd}" ]; then  
      sudo apt install shellcheck 
    fi

}

function run_cpp_lint() {

    # Add cpplint rule to BUILD files that do not contain it.
    for file in $(find cyber modules -name BUILD | \
      grep -v gnss/third_party | grep -v modules/teleop/encoder/nvenc_sdk6 | \
      xargs grep -l -E 'cc_library|cc_test|cc_binary' | xargs grep -L 'cpplint()')
    do
      sed -i '1i\load("//tools:cpplint.bzl", "cpplint")\n' $file
      sed -i -e '$a\\ncpplint()' $file
    done
    BUILD_TARGETS="`bazel query //modules/... except //modules/tools/visualizer/... union //cyber/...`"
    ${TEST_CMD} --config=cpplint -c dbg $BUILD_TARGETS

}

function run_bash_lint() {

    check_shellcheck
    FILES=$(find "${APOLLO_ROOT_DIR}/cyber"  -type f -name "*.sh" | grep -v ros)
    echo "${FILES}" | xargs shellcheck

}

function run_python_lint() {

    check_pyflakes
    find ${APOLLO_ROOT_DIR}/modules ${APOLLO_ROOT_DIR}/cyber \
      ${APOLLO_ROOT_DIR}/docker \
      ${APOLLO_ROOT_DIR}/tools \
      ${APOLLO_ROOT_DIR}/third_party \
      -type f -name "*.py" -exec pyflakes {} \;

}

function run_lint() {

  local cmd=$1
  case $cmd in
    python)
      run_python_lint
      if [ $? -eq 0 ]; then
        success 'Python Lint passed!'
      else
        fail 'Python Lint failed!'
      fi
      ;;
    cpp)
      run_cpp_lint
      if [ $? -eq 0 ]; then
        success 'CPP Lint passed!'
      else
        fail 'CPP Lint failed!'
      fi
      ;;
    shell)
      run_bash_lint
      if [ $? -eq 0 ]; then
        success 'SHELL Lint passed!'
      else
        fail 'SHELL Lint failed!'
      fi
      ;;
    *)
      run_python_lint
      run_cpp_lint
      run_bash_lint
      ;;
  esac
  
}
run_lint $@
