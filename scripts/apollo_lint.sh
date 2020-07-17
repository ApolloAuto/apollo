#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
# shellcheck source=apollo/scripts/apollo.bashrc
source "${TOP_DIR}/scripts/apollo.bashrc"

: ${STAGE:=dev}

function _cpp_lint_impl() {
    bazel test --distdir="${APOLLO_CACHE_DIR}" --config=cpplint "$@"
}

function run_cpp_lint() {
    pushd "${APOLLO_ROOT_DIR}" >/dev/null
        local cpp_dirs="cyber"
        if [[ "${STAGE}" == "dev" ]]; then
            cpp_dirs="${cpp_dirs} modules"
        fi
        # -not \( -path "modules/drivers/gnss/third_party" -prune \) \
        for prey in $(find ${cpp_dirs} -name BUILD \
                | xargs grep -l -E 'cc_library|cc_test|cc_binary|cuda_library' \
                | xargs grep -L 'cpplint()' ); do
            warning "unattended BUILD file found: ${prey}. Add cpplint() automatically."
            sed -i '1i\load("//tools:cpplint.bzl", "cpplint")\n' "${prey}"
            sed -i -e '$a\\ncpplint()' "${prey}"
            local buidifier
            buidifier="$(command -v buildifier)"
            if [ ! -z "${buidifier}" ]; then
                ${buidifier} -lint=fix "${prey}"
            fi
        done
    popd >/dev/null

    local targets="//cyber/..."
    _cpp_lint_impl "${targets}"

    if [[ "${STAGE}" == "dev" ]]; then
        _cpp_lint_impl "//modules/..."
    fi

}

function run_sh_lint() {
    local shellcheck_cmd
    shellcheck_cmd="$(command -v shellcheck)"
    if [ -z "${shellcheck_cmd}" ]; then
        warning "Command 'shellcheck' not found. For Debian/Ubuntu systems," \
                "please run the following command to install it: "
        warning "  sudo apt-get -y update"
        warning "  sudo apt-get -y install shellcheck"
        exit 1
    fi
    local sh_dirs="cyber scripts docker tools"
    if [[ "${STAGE}" == "dev" ]]; then
        sh_dirs="modules ${sh_dirs}"
    fi

    sh_dirs=$(printf "${APOLLO_ROOT_DIR}/%s " ${sh_dirs})
    run find ${sh_dirs} -type f \( -name "*.sh" -or -name "*.bashrc" \) -exec \
        shellcheck -x {} +

    for script in ${APOLLO_ROOT_DIR}/*.sh ; do
        run shellcheck -x "${script}"
    done
}

function run_py_lint() {
    local flake8_cmd
    flake8_cmd="$(command -v flake8)"
    if [ -z "${flake8_cmd}" ]; then
        warning "Command flake8 not found. You can install it manually via:"
        warning "  '[sudo -H] python3 -m pip install flake8'"
        exit 1
    fi

    local py_dirs="cyber docker tools"
    if [[ "${STAGE}" == "dev" ]]; then
        py_dirs="modules ${py_dirs}"
    fi

    py_dirs=$(printf "${APOLLO_ROOT_DIR}/%s " ${py_dirs})
    run find ${py_dirs} -type f \( -name "*.py" \) -exec \
        flake8 {} \;
}

function print_usage() {
    info "Usage: $0 [Options]"
    info "Options:"
    info "${TAB}-h|--help   Show this message and exit"
    info "${TAB}py|python   Lint Python code"
    info "${TAB}sh|shell    Lint Bash code"
    info "${TAB}cpp         Lint cpp code"
    info "${TAB}all         Lint all (C++/Python/Bash)"
    info "${TAB}help        Same as '--help'"
}

function run_lint() {
    local cmd="$1"
    case "${cmd}" in
    py|python)
        run_py_lint
        ;;
    cpp)
        run_cpp_lint
        ;;
    sh|shell)
        run_sh_lint
        ;;
    all)
        run_cpp_lint
        run_py_lint
        run_sh_lint
        ;;
    help)
        print_usage
        exit 0
        ;;
    *)
        print_usage
        exit 1
    esac
}

function main() {
    run_lint "$@"
}

main "$@"
