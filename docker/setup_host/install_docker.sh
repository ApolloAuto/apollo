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
function install_filesystem_support() {
  MACHINE_VERSION=$(uname -r)
  if [ "$MACHINE_VERSION" == "4.4.32-apollo-2-RT" ]; then
    echo "system have install realtime kernel"
    echo "it support overlay2, no need to install aufs"
    sudo modprobe overlay
  else
    MAIN_KERNEL_VERSION=${MACHINE_VERSION:0:1}
    if [ ${MAIN_KERNEL_VERSION} -gt 3 \
        -a -f /lib/modules/$MACHINE_VERSION/kernel/fs/overlayfs/overlay.ko ]; then
      echo "the kernel version 4 or higher;"
      echo "it has support overlay2"
      sudo modprobe overlay
    else
      echo "the kernel version is lower than 4"
      echo "try to install aufs"
      sudo apt-get update
      sudo apt-get install -y \
          linux-image-extra-${MACHINE_VERSION} \
          linux-image-extra-virtual
    fi
  fi

  sudo apt-get update
  sudo apt-get install -y \
      apt-transport-https \
      ca-certificates \
      curl \
      software-properties-common
  curl -fsSL "https://download.docker.com/linux/ubuntu/gpg" | sudo apt-key add -
}

function install_docker_x86() {
  sudo add-apt-repository \
     "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
  sudo apt-get update
  sudo apt-get install -y docker-ce
  sudo groupadd docker
  sudo gpasswd -a $USER docker
  newgrp docker
}

function install_docker_arm() {
  echo "deb [arch=arm64] https://download.docker.com/linux/ubuntu xenial edge" | \
      sudo tee /etc/apt/sources.list.d/docker.list
  sudo apt-get update
  sudo apt-get install -y docker-ce
  sudo groupadd docker
  sudo gpasswd -a $USER docker
  newgrp docker
}

function install() {
  # the machine type, currently support x86_64, aarch64
  install_filesystem_support
  MACHINE_ARCH=$(uname -m)
  if [ "$MACHINE_ARCH" == 'x86_64' ]; then
    install_docker_x86
  elif [ "$MACHINE_ARCH" == 'aarch64' ]; then
    install_docker_arm
  else
    echo "Unknown machine architecture $MACHINE_ARCH"
    exit 1
  fi
}

case $1 in
  install)
    install
    ;;
  uninstall)
    sudo apt-get remove docker docker-engine docker.io
    sudo apt-get purge docker-ce
    ;;
  *)
    install
    ;;
esac
