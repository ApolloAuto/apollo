#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# Fail on first error.
set -e

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh

find_src_file_checksum() {
  filename="${1}"
  # checksum file with format: <filename> <filesize> <checksum>
  checksum_file="${FILESDIR}/manifest"
  if grep -q "${filename}" "${checksum_file}"; then
    row=$(grep "${filename}" "${checksum_file}")
    echo "${row}" | awk '{print $3}'
  fi
}

variables_prepare() {
  if [[ -z "${INSTALL_ATOM}" ]]; then
    error "INSTALL_ATOM is not set. exiting"
    exit 1
  fi
  # variables can be overridden by the caller
  PF="${INSTALL_ATOM}"
  # PN="${PF%%-*}"
  PN="$(echo ${INSTALL_ATOM} | sed 's:-[0-9][0-9a-z._]\+\(-r[0-9]\+\)\?::')"
  # PVR="${PF#*-}"
  PVR="${PF#"${PN}-"}"
  if [[ "${PF}" =~ .*-r[0-9]+ ]]; then
    PV="${PVR%-*}"
    PR="${PF##*-}"
  else
    PV="${PVR}"
    PR="r0"
  fi
  P="${PN}-${PV}"
  DISTDIR="${DISTDIR:-/var/cache/apollo/distfiles}"
  BUILDDIR="${TMPDIR:-/var/tmp/apollo}/${PF}"
  WORKDIR="${BUILDDIR}/work"
  T="${BUILDDIR}/temp"
  FILESDIR="${SCRIPT_DIR}"
  for var in SRC_URI INSTALL_PREFIX; do
    real_var="$(echo "${PN}" | tr '[:lower:]-' '[:upper:]_')_${var}"
    if [[ ! -z "${!real_var}" ]]; then
      eval "${var}=${!real_var}"
    fi
  done
  INSTALL_PREFIX="${INSTALL_PREFIX:-/usr/local}"
}

git_checkout() {
  repo_uri="${1}"
  checkout_path="${2}"
  opts=()
  if [[ -n "${GIT_BRANCH}" ]]; then
    opts+=("--branch" "${GIT_BRANCH}")
  fi
  if [[ -n "${GIT_DEPTH}" ]]; then
    opts+=("--depth" "${GIT_DEPTH}")
  fi
  if [[ "${GIT_RECURSIVE}" == "true" || "${GIT_RECURSIVE}" == "1" ]]; then
    opts+=("--recursive")
  fi
  git clone "${opts[@]}" "${repo_uri}" "${checkout_path}"

  if [[ -n "${GIT_COMMIT}" ]]; then
    pushd "${checkout_path}"
    git checkout "${GIT_COMMIT}"
    popd
  fi

  if [[ -n "${GIT_SUBMODULES}" && -z "${GIT_RECURSIVE}" ]]; then
    pushd "${checkout_path}"

    total_submodules=($(git config --file .gitmodules --name-only --get-regexp path | awk -F'.' '{print $2}'))
    init_submodules=()
    for rule in "${GIT_SUBMODULES[@]}"; do
      if [[ $rule =~ -.* ]]; then
        del_modules=()
        for submodule in "${init_submodules[@]}"; do
          if [[ ${submodule} == ${rule:1} ]]; then
            del_modules+=("${submodule}")
          fi
        done
        for del_module in "${del_modules[@]}"; do
          init_submodules=("${init_submodules[@]/$del_module/}")
        done
      else
        for submodule in "${total_submodules[@]}"; do
          if [[ ${submodule} == ${rule} ]]; then
            init_submodules+=("${submodule}")
          fi
        done
      fi
    done
    git submodule update --init --recursive "${init_submodules[@]}"

    popd
  fi
}

git_src_unpack() {
  # only support one module, if more than one, please use invoke multiple times
  if [[ -z "${GIT_REPO_URI}" ]]; then
    # no need to checkout
    return
  fi

  rm -rf "${WORKDIR}/${PF}"
  git_checkout "${GIT_REPO_URI}" "${WORKDIR}/${PF}"

}

general_src_prepare() {
  mkdir -p "${DISTDIR}"
  local last_item=""
  DOWNLOAD_LIST=()
  FILENAME_LIST=()
  CHECKSUM_LIST=()
  item_count=0
  for item in $(echo ${SRC_URI}); do
    if [[ "${item}" == "->" ]]; then
      :
    else
      if [[ "${last_item}" == "->" ]]; then
        FILENAME_LIST[$((item_count - 1))]="${item}"
        CHECKSUM_LIST[$((item_count - 1))]="$(find_src_file_checksum ${FILENAME_LIST[$((item_count - 1))]})"
      else
        DOWNLOAD_LIST[${item_count}]="${item}"
        FILENAME_LIST[${item_count}]="$(basename ${item})"
        CHECKSUM_LIST[${item_count}]="$(find_src_file_checksum ${FILENAME_LIST[${item_count}]})"
        item_count=$((item_count + 1))
      fi
    fi
    last_item="${item}"
  done
  for i in $(seq 0 $((item_count - 1))); do
    download_if_not_cached \
      "${DISTDIR}/${FILENAME_LIST[${i}]}" \
      "${CHECKSUM_LIST[${i}]}" \
      "${DOWNLOAD_LIST[${i}]}"
  done
}

general_src_unpack() {
  for i in $(seq 0 $((item_count - 1))); do
    # TODO: support other archive types
    mkdir -p "${WORKDIR}/${PF}" && tar xf "${DISTDIR}/${FILENAME_LIST[${i}]}" --strip-component=1 -C "${WORKDIR}/${PF}"
  done
}

general_src_patch() {
  for patch_file in ${PATCHES[@]}; do
    patch -d "${WORKDIR}/${PF}" -p1 < "${patch_file}"
  done
}

general_src_configure() {
  pushd "${WORKDIR}/${PF}"
  ./configure \
    --prefix="${INSTALL_PREFIX}"
  popd
}

general_src_compile() {
  pushd "${WORKDIR}/${PF}"
  make -j$(nproc)
  popd
}

general_src_install() {
  pushd "${WORKDIR}/${PF}"
  make install
  popd
}

general_src_clean() {
  rm -rf "${BUILDDIR}"
}

src_prepare() {
  general_src_prepare
}

src_unpack() {
  general_src_unpack

  git_src_unpack
}

src_patch() {
  general_src_patch
}

src_configure() {
  general_src_configure
}

src_compile() {
  general_src_compile
}

src_install() {
  general_src_install
}

src_clean() {
  general_src_clean
}

pkg_install() {
  libpath="${INSTALL_PREFIX}/lib"
  if [[ -d "${libpath}" && -f "${APOLLO_LD_FILE}" ]]; then
    if ! grep -q "${libpath}" "${APOLLO_LD_FILE}"; then
      echo "${libpath}" >> "${APOLLO_LD_FILE}"
    fi
  fi
  ldconfig
}

pkg_install_post() {
  :
}

install_package() {
  # TODO: add checkers
  [[ $(type -t src_prepare_pre) == function ]] && src_prepare_pre
  src_prepare
  [[ $(type -t src_prepare_post) == function ]] && src_prepare_post

  [[ $(type -t src_unpack_pre) == function ]] && src_unpack_pre
  src_unpack
  [[ $(type -t src_unpack_post) == function ]] && src_unpack_post

  [[ $(type -t src_patch_pre) == function ]] && src_patch_pre
  src_patch
  [[ $(type -t src_patch_post) == function ]] && src_patch_post

  [[ $(type -t src_configure_pre) == function ]] && src_configure_pre
  src_configure
  [[ $(type -t src_configure_post) == function ]] && src_configure_post

  [[ $(type -t src_compile_pre) == function ]] && src_compile_pre
  src_compile
  [[ $(type -t src_compile_post) == function ]] && src_compile_post

  [[ $(type -t src_install_pre) == function ]] && src_install_pre
  src_install
  [[ $(type -t src_install_post) == function ]] && src_install_post

  # TODO: add full pkg install phase
  [[ $(type -t pkg_install_pre) == function ]] && pkg_install_pre
  pkg_install
  [[ $(type -t pkg_install_post) == function ]] && pkg_install_post

  [[ $(type -t src_clean_pre) == function ]] && src_clean_pre
  src_clean
  [[ $(type -t src_clean_post) == function ]] && src_clean_post

  return 0
}

variables_prepare
