#!/usr/bin/env bash


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}"

# the machine type, currently support x86_64, aarch64
MACHINE_ARCH=$(uname -m)
source ${DIR}/scripts/apollo_base.sh

TIME=$(date  +%Y%m%d_%H%M)
if [ -z "${DOCKER_REPO}" ]; then
    DOCKER_REPO=apolloauto/apollo
fi

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'

  echo -e "\n${RED}Usage${NONE}:
  .${BOLD}/apollo_docker.sh${NONE} [OPTION]"

  echo -e "\n${RED}Options${NONE}:
  ${BLUE}build${NONE}: run build only
  ${BLUE}build_opt${NONE}: build optimized binary for the code
  ${BLUE}build_gpu${NONE}: run build only with Caffe GPU mode support
  ${BLUE}build_opt_gpu${NONE}: build optimized binary with Caffe GPU mode support
  ${BLUE}build_fe${NONE}: compile frontend javascript code, this requires all the node_modules to be installed already
  ${BLUE}buildify${NONE}: fix style of BUILD files
  ${BLUE}check${NONE}: run build/lint/test, please make sure it passes before checking in new code
  ${BLUE}clean${NONE}: run Bazel clean
  ${BLUE}config${NONE}: run configurator tool
  ${BLUE}coverage${NONE}: generate test coverage report
  ${BLUE}doc${NONE}: generate doxygen document
  ${BLUE}lint${NONE}: run code style check
  ${BLUE}usage${NONE}: print this menu
  ${BLUE}release${NONE}: build release version
  ${BLUE}test${NONE}: run all unit tests
  ${BLUE}version${NONE}: display current commit and date
  ${BLUE}push${NONE}: pushes the images to Docker hub
  ${BLUE}gen${NONE}: release a docker release image
  "
}

function start_build_docker() {
  docker ps --format "{{.Names}}" | grep apollo_dev 1>/dev/null 2>&1
  if [ $? != 0 ]; then
    bash docker/scripts/dev_start.sh
  fi
}

function gen_docker() {
  IMG="apolloauto/apollo:run-${MACHINE_ARCH}-20171025_1428"
  RELEASE_DIR=${HOME}/.cache/release
  RELEASE_NAME="${DOCKER_REPO}:release-${MACHINE_ARCH}-${TIME}"
  DEFAULT_NAME="${DOCKER_REPO}:release-${MACHINE_ARCH}-latest"
  docker pull $IMG

  docker ps -a --format "{{.Names}}" | grep 'apollo_release' 1>/dev/null
  if [ $? == 0 ];then
    docker stop apollo_release 1>/dev/null
    docker rm -f apollo_release 1>/dev/null
  fi
  docker run -it \
      -d \
      --name apollo_release \
      --net host \
      -v "$RELEASE_DIR:/root/mnt" \
      -w /apollo \
      "$IMG"

  docker exec apollo_release bash -c "cp -Lr /root/mnt/* ."
  CONTAINER_ID=$(docker ps | grep apollo_release | awk '{print $1}')
  docker commit "$CONTAINER_ID" "$RELEASE_NAME"
  docker commit "$CONTAINER_ID" "$DEFAULT_NAME"
  docker stop "$CONTAINER_ID"
}

function push() {
  local DEFAULT_NAME="${DOCKER_REPO}:release-${MACHINE_ARCH}-latest"
  local RELEASE_NAME="${DOCKER_REPO}:release-${MACHINE_ARCH}-${TIME}"
  docker tag "$DEFAULT_NAME" "$RELEASE_NAME"
  docker push "$DEFAULT_NAME"
  docker push "$RELEASE_NAME"
}

if [ $# == 0 ];then
    print_usage
    exit 1
fi

start_build_docker

case $1 in
  print_usage)
    print_usage
    ;;
  push)
    push
    ;;
  gen)
    gen_docker
    ;;
  *)
    docker exec -u $USER apollo_dev bash -c "./apollo.sh $@"
    ;;
esac
