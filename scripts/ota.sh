#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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

APOLLO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

function update() {
  UPDATE_TAG=$(python ${APOLLO_ROOT}/modules/tools/ota/query_client.py)
  if [ "$?" != "0" ]; then
    echo $UPDATE_TAG
    exit 1
  fi

  tip="Type 'y' or 'Y' to start upgrade, or type any other key to exit"
  echo $tip
  read -n 1 user_agreed
  if [ "$user_agreed" != "y" ] && [ "$user_agreed" != "Y" ]; then
    exit 1
  fi
  cp ${APOLLO_ROOT}/scripts/ota.sh /home/$DOCKER_USER/.cache/
  ssh $DOCKER_USER@localhost  bash /home/$DOCKER_USER/.cache/ota.sh download $UPDATE_TAG
  python ${APOLLO_ROOT}/modules/tools/ota/verify_client.py
  if [ "$?" != "0" ]; then
    exit 1
  fi

  if [ -e "$HOME/.cache/apollo_release" ]; then
    rm -rf "$HOME/.cache/apollo_release"
  fi
  tar xzf /home/$DOCKER_USER/.cache/apollo_release.tar.gz -C /home/$DOCKER_USER/.cache
  NEW_TAG="${UPDATE_TAG}-local"

  ssh $DOCKER_USER@localhost  bash /home/$DOCKER_USER/.cache/ota.sh setup $NEW_TAG
  python ${APOLLO_ROOT}/modules/tools/ota/update_client.py ${UPDATE_TAG}
}

function clean() {
  rm -rf $HOME/.cache/apollo_update
  rm -rf $HOME/.cache/apollo_release.tar.gz
  rm -rf $HOME/.cache/sec_apollo_release.tar.gz
  rm -rf $HOME/.cache/ota.sh
  docker stop test_container 1>/dev/null
  docker rm test_container 1>/dev/null
}

function setup() {
  docker exec test_container cp -Lr /root/mnt/apollo_release/apollo /
  docker commit test_container $1
  echo "Please restart release docker with new release image: $1"
  clean
}

function download() {
  UPDATE_TAG=$1
  docker pull $UPDATE_TAG
  if [ "$?" != "0" ]; then
    echo "Downloading fails!"
    exit 1
  else
    echo "New release image has been downloaded!"
  fi
  docker ps -a --format "{{.Names}}" | grep 'test_container' 1>/dev/null
  if [ $? == 0 ]; then
      docker stop test_container 1>/dev/null
      docker rm -f test_container 1>/dev/null
  fi
  docker run -d -it --name test_container -v $HOME/.cache:/root/mnt $UPDATE_TAG
  docker exec test_container cp /root/sec_apollo_release.tar.gz /root/mnt
}

case $1 in
  update)
    update
    ;;
  download)
    download $2
    ;;
  setup)
    setup $2
    ;;
  *)
    echo "Usage: ota.sh update"
    ;;
esac
