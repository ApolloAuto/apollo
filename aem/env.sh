#!/bin/bash
#
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

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # being sourced
  export AEM_HOME=${SCRIPT_REAL_DIR:-/usr}
else
  # being run
  export AEM_HOME=${AEM_HOME:-$(dirname $(realpath $0))}
fi
export AEM_SYS_SHARE=${AEM_SYS_SHARE:-${AEM_HOME}/share/aem}
export AEM_VERSION='10.0.0-rc1-r3'

export APOLLO_ENVS_ROOT=${APOLLO_ENVS_ROOT:-${HOME}/.aem/envs}
export APOLLO_ENV_NAME="${USER}"
export APOLLO_ENV_WORKSPACE="${APOLLO_ENV_WORKSPACE:-${PWD}}"
export APOLLO_ENV_WORKROOT="${APOLLO_ENV_WORKROOT:-/apollo_workspace}"
export APOLLO_ENV_ARGS=()
export APOLLO_ENV_BACKEND="${APOLLO_ENV_BACKEND:-docker}"
export APOLLO_ENV_CONTAINER_PREFIX=${APOLLO_ENV_CONTAINER_PREFIX:-apollo_neo_dev_}
export APOLLO_ENV_CONTAINER_REPO_X86='registry.baidubce.com/apollo/apollo-env-gpu'
export APOLLO_ENV_CONTAINER_REPO_ARM='registry.baidubce.com/apollo/apollo-env-arm'
export APOLLO_ENV_CONTAINER_TAG_X86='10.0-u22'
export APOLLO_ENV_CONTAINER_TAG_ARM='10.0-u20'
export APOLLO_ENV_CONTAINER_NAME="${APOLLO_ENV_CONTAINER_NAME:-${APOLLO_ENV_CONTAINER_PREFIX}${APOLLO_ENV_NAME}}"
export APOLLO_ENV_CONTAINER_USER="${APOLLO_ENV_CONTAINER_USER:-${USER}}"
export APOLLO_ENV_CONTAINER_GROUP="${APOLLO_ENV_CONTAINER_GROUP:-$(id -gn)}"
export APOLLO_ENV_CONTAINER_UID=${APOLLO_ENV_CONTAINER_UID:-$(id -u)}
export APOLLO_ENV_CONTAINER_GID=${APOLLO_ENV_CONTAINER_GID:-$(id -g)}
export APOLLO_ENV_CONTAINER_SHELL="${APOLLO_ENV_CONTAINER_SHELL:-/bin/bash}"
export APOLLO_ENV_CROSS_PLATFORM=${APOLLO_ENV_CROSS_PLATFORM}
export APOLLO_ENV_USE_GPU_HOST=${APOLLO_ENV_USE_GPU_HOST:-1}

export AEM_FLAG_FORCE_RECREATE=${AEM_FLAG_FORCE_RECREATE:-0}
export AEM_FLAG_FORCE_RECREATE_VOLUMES=${AEM_FLAG_FORCE_RECREATE_VOLUMES:-0}
export AEM_FLAG_USE_LOCAL_IMAGE=${AEM_FLAG_USE_LOCAL_IMAGE:-0}
export AEM_FLAG_MOUNTS=()
export AEM_FLAG_USE_GPU=${AEM_FLAG_USE_GPU:-1}
export AEM_FLAG_SHM_SIZE=${AEM_FLAG_SHM_SIZE:-4G}

export AEM_GEO=${AEM_GEO}

# old env
# TODO: will be removed in the future
export USE_GPU_HOST=${USE_GPU_HOST:-0}
