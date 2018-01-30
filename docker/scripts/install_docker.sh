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

function install_docker_x86() {
  sudo apt-get -y install curl \
      "linux-image-extra-$(uname -r)" \
      linux-image-extra-virtual

  sudo apt-get -y install apt-transport-https ca-certificates

  curl -fsSL https://yum.dockerproject.org/gpg | sudo apt-key add -

  sudo add-apt-repository \
      "deb https://apt.dockerproject.org/repo/ \
         ubuntu-$(lsb_release -cs) \
         main"

  sudo apt-get update
  sudo apt-get -y --force-yes install docker-engine
  sudo usermod -aG docker "$USER"
}

function install_docker_arm() {
  sudo apt-get update
  sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common

  curl -fsSL "https://download.docker.com/linux/ubuntu/gpg" | sudo apt-key add -
  sudo bash -c 'echo "deb [arch=arm64] https://download.docker.com/linux/ubuntu xenial edge" > /etc/apt/sources.list.d/docker.list'

  sudo apt-get update
  sudo apt-get install -y docker-ce
}

# the machine type, currently support x86_64, aarch64
MACHINE_ARCH=$(uname -m)
if [ "$MACHINE_ARCH" == 'x86_64' ]; then
  install_docker_x86
elif [ "$MACHINE_ARCH" == 'aarch64' ]; then
  install_docker_arm
else
  echo "Unknown machine architecture $MACHINE_ARCH"
  exit 1
fi
