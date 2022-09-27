#!/bin/bash

# Copyright (c) 2021 LG Electronics, Inc. All Rights Reserved

DOCKER_REPO=apolloauto/apollo
TARGET_ARCH=x86_64
IMAGE_VERSION=18.04-20220803_1505
DEV_IMAGE=${DOCKER_REPO}:dev-${TARGET_ARCH}-${IMAGE_VERSION}
RUNTIME_IMAGE=${DOCKER_REPO}:runtime-${TARGET_ARCH}-${IMAGE_VERSION}
STANDALONE_IMAGE=${DOCKER_REPO}:standalone-${TARGET_ARCH}-${IMAGE_VERSION}
STANDALONE_IMAGE_LATEST=${DOCKER_REPO}:standalone-${TARGET_ARCH}-18.04-6.1-latest

set -e

if [ "$1" == "rebuild" ] ; then
  docker/scripts/dev_start.sh -y
  docker exec -u $USER -t apollo_dev_$USER bazel clean --expunge || true
  docker exec -u $USER -t apollo_dev_$USER /apollo/apollo.sh release --clean --resolve
fi

if [ "$1" == "reinstall" ] ; then
  docker exec -u $USER -t apollo_dev_$USER /apollo/apollo.sh release --clean --resolve
fi

rm -rf docker/build/output

# Expects that the Apollo was already built in apollo_dev_$USER
if ! [ -f output/syspkgs.txt ] ; then
  echo "ERROR: this directory doesn't have output/syspkgs.txt"
  echo "       make sure apollo was built with \"/apollo/apollo.sh release --resolve\""
  echo "       or add \"rebuild\" parameter to this script"
  echo "       and it will be built automatically"
  exit 1
fi

cp output/syspkgs.txt docker/build/
docker build \
    --build-arg DEV_IMAGE_IN=${DEV_IMAGE} \
    --build-arg GEOLOC=us \
    --build-arg CUDA_LITE=11.1 \
    --build-arg CUDNN_VERSION=8.0.4.30 \
    --build-arg TENSORRT_VERSION=7.2.1 \
    -f docker/build/runtime.x86_64.dockerfile.sample \
    docker/build/ \
    -t ${RUNTIME_IMAGE}
rm -f docker/build/syspkgs.txt

mv output docker/build

docker build \
    --build-arg BASE_IMAGE=${RUNTIME_IMAGE} \
    -f docker/build/standalone.x86_64.dockerfile \
    docker/build/ \
    -t ${STANDALONE_IMAGE}

docker tag ${STANDALONE_IMAGE} ${STANDALONE_IMAGE_LATEST}

/bin/echo -e "Docker image with prebuilt files was built and tagged as ${STANDALONE_IMAGE}, you can start it with: \n\
  bash docker/scripts/runtime_start.sh --standalone --local --tag standalone-${TARGET_ARCH}-${IMAGE_VERSION}\n\
and switch into it with:\n\
  bash docker/scripts/runtime_into_standalone.sh"
