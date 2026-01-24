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

DOCKER_USER="${USER}"
CYBER_CONTAINER="apollo_cyber_${USER}"

APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${APOLLO_ROOT_DIR}/scripts/apollo.bashrc"

xhost +local:root 1>/dev/null 2>&1

HOST_ARCH="$(uname -m)"
TARGET_ARCH="$(docker exec "${CYBER_CONTAINER}" uname -m)"

IFS='' read -r -d '' NONROOT_SUDO_ERRMSG <<EOF
"sudo: effective uid is not 0, is /usr/bin/sudo on a file system with the \
'nosuid' option set or an NFS file system without root privileges?"
EOF

if [[ "${HOST_ARCH}" != "${TARGET_ARCH}" ]]; then
    warning "We only tested aarch containers running on x86_64 hosts." \
        "And after we changed from ROOT to NON-ROOT users, executing" \
        "sudo operations complains:\n  " \
        "${NONROOT_SUDO_ERRMSG}"
fi

if [[ "${TARGET_ARCH}" == "x86_64" || "${TARGET_ARCH}" == "aarch64" ]]; then
    docker exec \
        -u "${DOCKER_USER}" \
        -it "${CYBER_CONTAINER}" \
        /bin/bash
else
    error "Unsupported architecture: ${TARGET_ARCH}"
    exit 1
fi

# Note(storypku): Tested on Ubuntu 18.04 running on Jetson TX2,
# The following steps are no longer needed.
# if [ "${TARGET_ARCH}" == "aarch64" ]; then
#    info "For the first time after CyberRT container starts, you can running" \
#         "the following two commands to su to a non-root user:"
#    info "1) /apollo/scripts/docker_start_user.sh"
#    info "2) su - ${DOCKER_USER}"
#
#    # warning "! To exit, please use 'ctrl+p ctrl+q' !"
#    # docker attach "${CYBER_CONTAINER}"
#    docker exec \
#        -u root \
#        -it "${CYBER_CONTAINER}" \
#        /bin/bash

xhost -local:root 1>/dev/null 2>&1
