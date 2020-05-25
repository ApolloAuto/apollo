#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#   ./build_cyber.sh [-l] -f <cyber.dockerfile> [-m <build|download>]
# E.g.,
#   ./build_cyber.sh -f ./cyber.aarch64.dockerfile -m download

# Fail on first error.
set -euo pipefail

SUPPORTED_ARCHS=" x86_64 aarch64 "
REPO=apolloauto/apollo
UBT_LTS="18.04"
LOCAL_DEV_TAG="${REPO}:local_cyber_dev"

STAGE="cyber"
HOST_ARCH="$(uname -m)"
TARGET_ARCH=

LOCAL_DEV_FLAG="no"
MODE="download"
GEOLOC="us"
DOCKERFILE=""
TAB="    "

function print_usage() {
    local prog_name=$(basename "$0")
    echo "Usage:"
    echo "${TAB}${prog_name} [-l] -f <cyber_dockerfile> [-m <build|download>] [-g <us|cn>]"
    echo "${TAB}${prog_name} -h/--help    # Show this message"
    echo "E.g.,"
    echo "${TAB}${prog_name} -f cyber.x86_64.dockerfile -m build"
    echo "${TAB}${prog_name} -l -f cyber.aarch64.dockerfile -m download"
}

function determine_target_arch() {
    local dockfile="$1"
    IFS='.' read -ra __arr <<< "${dockfile}"
    if [[ ${#__arr[@]} -ne 3 ]]; then
        echo "Expected dockerfile with name [prefix_]<target>.<arch>.dockerfile"
        echo "Got ${dockfile}. Exiting..."
        exit 1
    fi
    IFS=' '

    local arch="${__arr[1]}"
    if [[ "${SUPPORTED_ARCHS}" != *" ${arch} "* ]]; then
        echo "Unsupported architecture: ${arch}. Allowed values:${SUPPORTED_ARCHS}"
        exit 1
    fi
    TARGET_ARCH="${arch}"
    return 0
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

    # Set and check target arch
    determine_target_arch "${DOCKERFILE}}"
    if [[ "${TARGET_ARCH}" != "${HOST_ARCH}" ]]; then
        echo "[WARNING] Host arch (${HOST_ARCH}) != Target Arch (${TARGET_ARCH}) " \
             "for dockerfile \"$DOCKERFILE\""
    fi
}

parse_arguments "$@"
check_arguments

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"
TIME=$(date +%Y%m%d_%H%M)
TAG="${REPO}:cyber-${TARGET_ARCH}-${UBT_LTS}-${TIME}"

echo "=====.=====.=====.=====  Docker Image Build for Cyber =====.=====.=====.====="
echo "|  Docker build ${TAG}"
echo "|  ${TAB}using dockerfile=${DOCKERFILE}"
echo "|  ${TAB}TARGET_ARCH=${TARGET_ARCH}, HOST_ARCH=${HOST_ARCH}"
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
