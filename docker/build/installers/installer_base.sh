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

RCFILES_DIR="/opt/apollo/rcfiles"
DEST_DIR_BASE="/opt/apollo/pkgs"
SYSROOT_DIR="/opt/apollo/sysroot"
ARCHIVE_DIR="/tmp/archive"

if [[ ! -d "${DEST_DIR_BASE}" ]]; then
    mkdir -p "${DEST_DIR_BASE}"
fi

if [[ ! -d "${SYSROOT_DIR}" ]]; then
    mkdir -p "${SYSROOT_DIR}"
fi

# sha256sum was provided by coreutils
function download_if_not_cached {
  local pkg_name=$1
  local checksum_expected=$2
  local url=$3
  local use_cache=0
  if [ -e "$ARCHIVE_DIR/$pkg_name" ]; then
    checksum_actual=$(sha256sum "$ARCHIVE_DIR/$pkg_name" | awk '{print $1}')
    if [ x"$checksum_actual" = x"$checksum_expected" ]; then
      info "package $pkg_name found in fscache, will use it."
      use_cache=1
    else
      warning "package $pkg_name found in fscache, but checksum mismatch."
      warning "    expecting $checksum_expected, got $checksum_actual."
    fi
  fi

  local my_schema
  my_schema=$(package_schema "$url" "$pkg_name")

  if [ $use_cache -eq 0 ]; then
    if [[ "$my_schema" == "http" ]]; then
      info "Manually download $pkg_name ..."
      wget "$url" -O "$pkg_name"
      ok "Successfully downloaded $pkg_name"
    elif [[ "$my_schema" == "git" ]]; then
      info "Clone into git repo $url..."
      git clone --recurse-submodules --single-branch "$url"
      ok "Successfully cloned git repo: $url"
    else
      error "Unknown schema for package \"$pkg_name\", url=\"$url\""
    fi
  else
    info "Congrats, cache hit ${pkg_name}, schema ${my_schema}, will use it."
    if [ "$my_schema" = "http" ]; then
      ln -s "$ARCHIVE_DIR/${pkg_name}" "$pkg_name"
    elif [ "$my_schema" = "git" ]; then
      tar xzf "$ARCHIVE_DIR/${pkg_name}"
    else
      error "Unknown schema for package \"$pkg_name\", url=\"$url\""
    fi
  fi
}

