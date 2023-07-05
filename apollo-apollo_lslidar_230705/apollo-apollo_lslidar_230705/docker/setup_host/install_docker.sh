#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

ARCH="$(uname -m)"

##==============================================================##
## Note(storypku): DRY broken for this self-contained script.
##==============================================================##
BOLD='\033[1m'
RED='\033[0;31m'
WHITE='\033[34m'
NO_COLOR='\033[0m'

function info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() {
  (echo >&2 -e "[${RED}ERROR${NO_COLOR}] $*")
}
##==============================================================##

function install_filesystem_support() {
  local kernel_version="$(uname -r)"
  if [ "$kernel_version" == "4.4.32-apollo-2-RT" ]; then
    info "Apollo realtime kernel ${kernel_version} found."
    sudo modprobe overlay
  else
    local kernel_version_major=${kernel_version:0:1}
    local overlay_ko_path="/lib/modules/$kernel_version/kernel/fs/overlayfs/overlay.ko"
    if [ "${kernel_version_major}" -ge 4 ] && [ -f "${overlay_ko_path}" ] ; then
      info "Linux kernel ${kernel_version} has builtin overlay2 support."
      sudo modprobe overlay
    elif [ ${kernel_version_major} -ge 4 ]; then
      error "Overlay kernel module not found at ${overlay_ko_path}." \
            "Are you running on a customized Linux kernel? "
      exit 1
    else
      error "Linux kernel version >= 4 expected. Got ${kernel_version}"
      exit 1
    fi
  fi
}

function install_prereq_packages() {
  sudo apt-get -y update
  sudo apt-get -y install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
}


function setup_docker_repo_and_install() {
  local issues_link="https://github.com/ApolloAuto/apollo/issues"
  local arch_alias=
  if [ "${ARCH}" == "x86_64" ]; then
    arch_alias="amd64"
  elif [ "${ARCH}" == "aarch64" ]; then
    arch_alias="arm64"
  else
    error "Currently, ${ARCH} support has not been implemented yet." \
          "You can create an issue at ${issues_link}."
    exit 1
  fi

  curl -fsSL "https://download.docker.com/linux/ubuntu/gpg" | sudo apt-key add -
  sudo add-apt-repository \
    "deb [arch=${arch_alias}] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) \
    stable"
  sudo apt-get update
  sudo apt-get install -y docker-ce \
    docker-ce-cli \
    containerd.io
}

function post_install_settings() {
  sudo usermod -aG docker $USER
  sudo systemctl restart docker
  # sudo groupadd docker
  # sudo gpasswd -a $USER docker
  # newgrp docker
}

function install_docker() {
  # Architecture support, currently: x86_64, aarch64
  install_filesystem_support
  install_prereq_packages
  setup_docker_repo_and_install
  post_install_settings
}

function uninstall_docker() {
  sudo apt-get -y remove docker docker-engine docker.io
  sudo apt-get purge docker-ce

  sudo sed -i '/download.docker.com/d' /etc/apt/sources.list
  sudo apt-key del 0EBFCD88
}

function main() {
  case $1 in
    install)
      install_docker
      ;;
    uninstall)
      uninstall_docker
      ;;
    *)
      install_docker
      ;;
  esac
}

main "$@"
