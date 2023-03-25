#!/usr/bin/env bash

BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

function info() {
    (>&2 echo -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() {
    (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() {
    (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() {
    (>&2 echo -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}

export RCFILES_DIR="/opt/apollo/rcfiles"
export APOLLO_DIST="${APOLLO_DIST:-stable}"

export PKGS_DIR="/opt/apollo/pkgs"
export SYSROOT_DIR="/opt/apollo/sysroot"

export APOLLO_PROFILE="/etc/profile.d/apollo.sh"
export APOLLO_LD_FILE="/etc/ld.so.conf.d/apollo.conf"
export DOWNLOAD_LOG="/opt/apollo/build.log"
export LOCAL_HTTP_ADDR="http://172.17.0.1:8388"

if [[ "$(uname -m)" == "x86_64" ]]; then
    export SUPPORTED_NVIDIA_SMS="5.2 6.0 6.1 7.0 7.5 8.0 8.6"
else # AArch64
    export SUPPORTED_NVIDIA_SMS="5.3 6.2 7.2"
fi

function py3_version() {
    local version
    # major.minor.rev (e.g. 3.6.9) expected
    version="$(python3 --version | awk '{print $2}')"
    echo "${version%.*}"
}

function pip3_install() {
    python3 -m pip install --timeout 30 --no-cache-dir $@
}

function apt_get_update_and_install() {
    # --fix-missing
    apt-get -y update && \
        apt-get -y install --no-install-recommends "$@"
}

function apt_get_remove() {
    apt-get -y purge --autoremove "$@"
}

# Ref: https://reproducible-builds.org/docs/source-date-epoch
function source_date_epoch_setup() {
    DATE_FMT="+%Y-%m-%d"
    export SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-$(date +%s)}"
    export BUILD_DATE=$(date -u -d "@$SOURCE_DATE_EPOCH" "$DATE_FMT" 2>/dev/null \
        || date -u -r "$SOURCE_DATE_EPOCH" "$DATE_FMT" 2>/dev/null \
        || date -u "$DATE_FMT")
}

function apollo_environ_setup() {
    if [ -z "${SOURCE_DATE_EPOCH}" ]; then
        source_date_epoch_setup
    fi

    if [ ! -d "${PKGS_DIR}" ]; then
        mkdir -p "${PKGS_DIR}"
    fi
    if [ ! -d "${SYSROOT_DIR}" ]; then
        mkdir -p ${SYSROOT_DIR}/{bin,include,lib,share}
    fi
    if [ ! -f "${APOLLO_LD_FILE}" ]; then
        echo "${SYSROOT_DIR}/lib" | tee -a "${APOLLO_LD_FILE}"
    fi
    if [ ! -f "${APOLLO_PROFILE}" ]; then
        cp -f /opt/apollo/rcfiles/apollo.sh.sample "${APOLLO_PROFILE}"
        echo "add_to_path ${SYSROOT_DIR}/bin" >> "${APOLLO_PROFILE}"
    fi
    if [ ! -f "${DOWNLOAD_LOG}" ]; then
        echo "##==== Summary: Apollo Package Downloads ====##" > "${DOWNLOAD_LOG}"
        echo -e "Package\tSHA256\tDOWNLOADLINK" | tee -a "${DOWNLOAD_LOG}"
    fi
}

apollo_environ_setup

# We only accept predownloaded git tarballs with format
# "pkgname.git.53549ad.tgz" or "pkgname_version.git.53549ad.tgz"
function package_schema {
    local __link=$1
    local schema="http"

    if [[ "${__link##*.}" == "git" ]] ; then
        schema="git"
        echo $schema
        return
    fi

    IFS='.' # dot(.) is set as delimiter

    local __pkgname=$2
    read -ra __arr <<< "$__pkgname" # Array of tokens separated by IFS
    if [[ ${#__arr[@]} -gt 3 ]] && [[ "${__arr[-3]}" == "git" ]] \
        && [[ ${#__arr[-2]} -eq 7 ]] ; then
        schema="git"
    fi
    IFS=' ' # reset to default value after usage

    echo "$schema"
}

function create_so_symlink() {
    local mydir="$1"
    for mylib in $(find "${mydir}" -name "lib*.so.*" -type f); do
        mylib=$(basename "${mylib}")
        ver="${mylib##*.so.}"
        if [ -z "$ver" ]; then
            continue
        fi
        libX="${mylib%%.so*}"
        IFS='.' read -ra arr <<< "${ver}"
        IFS=" " # restore IFS
        ln -s "${mylib}" "${mydir}/${libX}.so.${arr[0]}"
        ln -s "${mylib}" "${mydir}/${libX}.so"
    done
}

function _local_http_cached() {
    if /usr/bin/curl -sfI "${LOCAL_HTTP_ADDR}/$1"; then
        return
    fi
    false
}

function _checksum_check_pass() {
    local pkg="$1"
    local expected_cs="$2"
    # sha256sum was provided by coreutils
    local actual_cs=$(/usr/bin/sha256sum "${pkg}" | awk '{print $1}')
    if [[ "${actual_cs}" == "${expected_cs}" ]]; then
        true
    else
        warning "$(basename ${pkg}): checksum mismatch, ${expected_cs}" \
                "exected, got: ${actual_cs}"
        false
    fi
}

function download_if_not_cached {
    local pkg_name="$1"
    local expected_cs="$2"
    local url="$3"

    echo -e "${pkg_name}\t${expected_cs}\t${url}" >> "${DOWNLOAD_LOG}"

    if _local_http_cached "${pkg_name}" ; then
        local local_addr="${LOCAL_HTTP_ADDR}/${pkg_name}"
        info "Local http cache hit ${pkg_name}..."
        wget "${local_addr}" -O "${pkg_name}"
        if _checksum_check_pass "${pkg_name}" "${expected_cs}"; then
            ok "Successfully downloaded ${pkg_name} from ${LOCAL_HTTP_ADDR}," \
               "will use it."
            return
        else
            warning "Found ${pkg_name} in local http cache, but checksum mismatch."
            rm -f "${pkg_name}"
        fi
    fi # end http cache check

    local my_schema
    my_schema=$(package_schema "$url" "$pkg_name")

    if [[ "$my_schema" == "http" ]]; then
        info "Start to download $pkg_name from ${url} ..."
        wget "$url" -O "$pkg_name"
        ok "Successfully downloaded $pkg_name"
    elif [[ "$my_schema" == "git" ]]; then
        info "Clone into git repo $url..."
        git clone  "${url}" --branch master --recurse-submodules --single-branch
        ok "Successfully cloned git repo: $url"
    else
        error "Unknown schema for package \"$pkg_name\", url=\"$url\""
    fi
}

USE_AMD_GPU=0
USE_NVIDIA_GPU=0

function determine_gpu_use_host() {
    if [[ "${TARGET_ARCH}" == "aarch64" ]]; then
        if lsmod | grep -q "^nvgpu"; then
            USE_NVIDIA_GPU=1
        fi
    elif [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        if [[ ! -x "$(command -v nvidia-smi)" ]]; then
            warning "No nvidia-smi found."
        elif [[ -z "$(nvidia-smi)" ]]; then
            warning "No NVIDIA GPU device found."
        else
            USE_NVIDIA_GPU=1
        fi
        if [[ ! -x "$(command -v rocm-smi)" ]]; then
            warning "No rocm-smi found."
        elif [[ -z "$(rocm-smi)" ]]; then
            warning "No AMD GPU device found."
        else
            USE_AMD_GPU=1
        fi
    else
        error "Unsupported CPU architecture: ${HOST_ARCH}"
        exit 1
    fi
}
