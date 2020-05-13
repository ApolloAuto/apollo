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
# Usage:
#   ./build_dev.sh [-l] -f <dev.dockerfile> [-m <build|download>] [ -g <us|cn> ]
# Where
#   -l      Also tag the newly built image as apolloauto/apollo:local_dev
#   -g      Support US/CN geographical settings
# E.g.,
#   ./build_dev.sh -f ./dev.x86_64.dockerfile -g cn
#
ARCH=$(uname -m)
REPO=apolloauto/apollo
LOCAL_DEV_TAG="${REPO}:local_dev"

LOCAL_DEV_FLAG="no"
MODE="download"
GEOLOC="us"
STAGE="dev"
DOCKERFILE=""
TAB="    "

function print_usage() {
    local prog_name=$(basename "$0")
    echo "Usage:"
    echo "${TAB}${prog_name} [-l] -f <dev_dockerfile> [-m <build|download>] [-g <us|cn>]"
    echo "${TAB}${prog_name} -h/--help    # Show this message"
    echo "E.g.,"
    echo "${TAB}${prog_name} -f dev.x86_64.dockerfile -m build"
    echo "${TAB}${prog_name} -l -f dev.aarch64.dockerfile -m download"
}

function parse_arguments() {
    if [[ $# -eq 0 ]] || [[ "$1" == "--help" ]]; then
        print_usage
        exit 0
    fi
    while getopts "hlf:m:g:" opt; do
        case $opt in
            l)
                LOCAL_DEV_FLAG="yes"
                ;;
            f)
                DOCKERFILE=$OPTARG
                ;;
            m)
                MODE=$OPTARG
                ;;
            g)
                GEOLOC=$OPTARG
                ;;
            h)
                print_usage
                exit 1
                ;;
            *)
                echo "Unknown option: -$opt"
                print_usage
                exit 1
                ;;
        esac
    done
}

function check_arguments() {
    if [[ "${MODE}" == "build" ]]; then
        echo "Build all dependencies from source code"
    elif [[ "${MODE}" == "download" ]]; then
        echo "Optimize installation of some dependencies from prebuilt packages"
    else
        echo "Installation mode \"$MODE\" not supported"
        exit 1
    fi
    if [[ "${GEOLOC}" == "cn" ]]; then
        echo "Docker image built for CN users"
    else
        GEOLOC="us"
    fi

    if [[ -z "${DOCKERFILE}" ]]; then
        echo "Dockfile not specified"
        exit 1
    fi
    if [[ "$DOCKERFILE" == *${ARCH}* ]]; then
        echo "Dockerfile to build: ${DOCKERFILE}"
    else
        echo "Dockerfile \"$DOCKERFILE\" doesn't match current architecture."
        exit 1
    fi
}

parse_arguments "$@"
check_arguments

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"
TIME=$(date +%Y%m%d_%H%M)
TAG="${REPO}:dev-${ARCH}-18.04-${TIME}"

# Fail on first error.
set -e
echo "=====.=====.=====.=====  Docker Image Build for Devel =====.=====.=====.====="
echo "|  Docker build ${TAG}"
echo "|  ${TAB}using dockerfile=${DOCKERFILE}"
echo "|  ${TAB}INSTALL_MODE=${MODE}, GEOLOC=${GEOLOC}"
echo "=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.====="

docker build -t "${TAG}" --build-arg INSTALL_MODE="${MODE}" \
    --build-arg GEOLOC="${GEOLOC}" \
    --build-arg BUILD_STAGE="${STAGE}" \
    -f "${DOCKERFILE}" "${CONTEXT}"
echo "Built new image ${TAG}"

if [[ "$LOCAL_DEV_FLAG" == "yes" ]]; then
    docker image tag "${TAG}" "${LOCAL_DEV_TAG}"
    echo "Also tagged as ${LOCAL_DEV_TAG}"
fi
