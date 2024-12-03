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

# Flags
#set -u
#set -e
#set -x

subenv() {
  template="${1}"
  target="${2}"
  eval "
cat <<EOF
$(< "${template}")
EOF
" > ${target} 2> /dev/null
}
export -f subenv

cmp_version() {
  local v1=$1
  local v2=$2
  if [[ $v1 == $v2 ]]; then
    echo 0
    return
  fi
  local vv="$(echo "${v1}\n${v2}" | sort -V | head -n1)"
  if [[ $vv == $v1 ]]; then
    echo -1
  else
    echo 1
  fi
}
export -f cmp_version

check_ip_valid() {
  if [[ "${1}" =~ ^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$ ]]; then
    return 0
  fi
  return 1
}

check_dns_host_valid() {
  if [[ "${1}" =~ ^(([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\-]*[a-zA-Z0-9])\.)+([A-Za-z]|[A-Za-z][A-Za-z0-9\-]*[A-Za-z0-9])$ ]]; then
    return 0
  fi
  return 1
}

check_nvidia_gpu_available() {
  if [[ "$(uname -m)" == "aarch64" ]]; then
    if lsmod | grep -q "^nvgpu"; then
      return 0
    else
      return 1
    fi
  elif [[ "$(uname -m)" == "x86_64" ]]; then
    if [[ ! -x "$(command -v nvidia-smi)" ]]; then
      warn "No nvidia-smi found."
      return 1
    elif [[ -z "$(nvidia-smi)" ]]; then
      warn "No NVIDIA GPU device found."
      return 1
    else
      return 0
    fi
  else
    return 1
  fi
}
export -f check_nvidia_gpu_available

check_amd_gpu_available() {
  if [[ "$(uname -m)" == "x86_64" ]]; then
    if [[ ! -x "$(command -v rocm-smi)" ]]; then
      warn "No rocm-smi found."
      return 1
    elif [[ -z "$(rocm-smi)" ]]; then
      warn "No AMD GPU device found."
      return 1
    else
      return 0
    fi
  else
    return 1
  fi
}
export -f check_amd_gpu_available

docker_volume_exists() {
  local volume="${1}"
  local query="$(docker volume ls -q -f name=${volume} | grep "^${volume}$")"
  if [[ "${query}" == "${volume}" ]]; then
    return 0
  else
    return 1
  fi
}

export -f docker_volume_exists

docker_image_exists() {
  local img="${1}"
  if docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${img}$"; then
    return 0
  fi
  return 1
}
export -f docker_image_exists

docker_pull() {
  local img="${1}"
  local name_0="${img##/*}"
  if ! check_dns_host_valid "${name_0}"; then
    # no registry specified, check if enable GEO_REGISTRY
    if [[ -n "${AEM_GEO_REGISTRY}" ]]; then
      img="${AEM_GEO_REGISTRY}/${img}"
    fi
  fi

  if [[ "${AEM_FLAG_USE_LOCAL_IMAGE}" == "1" ]] && docker_image_exists "${img}"; then
    info "docker image ${img} already exists, skip pulling"
    return 0
  fi
  info "pulling docker image ${img} ..."
  if ! docker pull "${img}"; then
    if docker_image_exists "${img}"; then
      warn "failed to pull image: ${img}, fallback to local image"
      return 0
    else
      error "failed to pull image: ${img}"
      return 1
    fi
  fi
}
export -f docker_pull

docker_setup_cross_platform() {
  platform="${1}"
  if [[ "${1}" == "aarch64" ]]; then
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes -c yes > /dev/null
    if [[ ! $? -eq 0 ]]; then
      error "qemu setup failed"
      return 1
    fi
  fi
  return 0
}

docker_container_exists() {
  local container="${1}"
  local query="$(docker ps -a --format={{.Names}} -f name=${container} | grep "^${container}$")"
  if [[ "${query}" == "${container}" ]]; then
    return 0
  else
    return 1
  fi
}
export -f docker_container_exists

docker_container_running() {
  local container="${1}"
  local query="$(docker ps --format={{.Names}} -f status=running -f name=${container} | grep "^${container}$")"
  if [[ "${query}" == "${container}" ]]; then
    return 0
  else
    return 1
  fi
}
export -f docker_container_running

docker_container_stopped() {
  local container="${1}"
  local query="$(docker ps --format={{.Names}} -f status=exited -f name=${container} | grep "^${container}$")"
  if [[ "${query}" == "${container}" ]]; then
    return 0
  else
    return 1
  fi
}
export -f docker_container_stopped

apollo_envhome_exists() {
  local env_name="${1}"
  if [[ -d "${APOLLO_ENVS_ROOT}/${env_name}" ]]; then
    return 0
  else
    return 1
  fi
}
export -f apollo_envhome_exists

apollo_env_exists() {
  local env_name="${1}"
  if ! apollo_envhome_exists "${env_name}"; then
    return 1
  fi
  if [[ -e "${APOLLO_ENVS_ROOT}/${env_name}/env.config" ]]; then
    declare backend
    backend="$(source "${APOLLO_ENVS_ROOT}/${env_name}/env.config" && echo "${APOLLO_ENV_BACKEND}")"
    if [[ "${backend}" == "host" ]]; then
      if [[ -f "${APOLLO_ENVS_ROOT}/${env_name}/inited" ]]; then
        return 0
      else
        return 1
      fi
    elif [[ "${backend}" == "docker" ]]; then
      local container_name="${APOLLO_ENV_CONTAINER_PREFIX}${env_name}"
      if docker_container_exists "${container_name}"; then
        return 0
      else
        return 1
      fi
    else
      fatal "unsupported backend: ${APOLLO_ENV_BACKEND}"
      exit 1
    fi
  else
    return 1
  fi
}
export -f apollo_env_exists

apollo_create_env() {
  env_name="${APOLLO_ENV_NAME}"
  if apollo_env_exists "${env_name}"; then
    if [[ "${AEM_FLAG_FORCE_RECREATE}" == 0 ]]; then
      return 0
    fi
  fi
  if [[ "${APOLLO_ENV_BACKEND}" == "host" ]]; then
    apollo_create_hostenv
  elif [[ "${APOLLO_ENV_BACKEND}" == "docker" ]]; then
    apollo_create_container
  else
    fatal "unsupported backend: ${APOLLO_ENV_BACKEND}"
    exit 1
  fi
}
export -f apollo_create_env

apollo_create_envhome() {
  env_name="${APOLLO_ENV_NAME}"
  if apollo_envhome_exists "${env_name}"; then
    warn "apollo environment \`${env_name}' already exists, enter directly. Or you can use flag \`--force' to recreate it."
    if [[ "${AEM_FLAG_FORCE_RECREATE}" == 0 ]]; then
      return 0
    fi
  fi

  env_home_global="${APOLLO_ENVS_ROOT}/${env_name}"
  env_home_local="${APOLLO_ENV_WORKSPACE}/.aem"
  env_root_global="${env_home_global}/envroot"
  env_root_local="${env_home_local}/envroot"
  mkdir -p "${env_home_global}"
  mkdir -p "${env_home_local}"
  if [[ "${APOLLO_ENV_WORKLOCAL}" == "1" ]]; then
    env_home="${env_home_local}"
    env_root="${env_root_local}"
    mkdir -p "${env_root_local}"/{apollo,etc,opt}
    if [[ -e "${env_root_global}" ]]; then
      warn "Symbolic link ${env_root_global} already exists. Remove it and create a new one."
      rm -rf "${env_root_global}"
    fi
    ln -snf "${env_root_local}" "${env_root_global}"
  else
    env_home="${env_home_global}"
    env_root="${env_root_global}"
    mkdir -p "${env_root}"/{apollo,etc,opt}
    ln -snf "${env_root_global}" "${env_root_local}"
  fi

  export APOLLO_ENV_HOME="${env_home}"
  export APOLLO_ENV_ROOT="${env_root}"

  # docker backend, create volume
  if [[ "${APOLLO_ENV_BACKEND}" == "docker" ]]; then
    for x in {apollo,opt}; do
      local volume_name="${APOLLO_ENV_CONTAINER_PREFIX}${env_name}_${x}"
      if [[ "${AEM_FLAG_FORCE_RECREATE_VOLUMES}" == "1" ]]; then
        if docker_volume_exists "${volume_name}"; then
          docker volume rm "${volume_name}"
        fi
      fi
      if ! docker_volume_exists "${volume_name}"; then
        info "creating docker volume: ${volume_name}"
        docker volume create \
          --driver local \
          --opt type=none \
          --opt device="${env_root}/${x}" \
          --opt o=bind \
          "${volume_name}" > /dev/null
      fi
    done
  fi
}
export -f apollo_create_envhome

apollo_create_hostenv() {
  install -D -m 644 "${AEM_HOME}/activate.sh" "${APOLLO_ENV_HOME}/activate.sh"

  # deploy apollo dependencies
  if [[ ${UID} == 0 ]]; then
    # running as root
    pip3 install pipx jinja2 requests
    apt install -y python3-apt python3-venv rsync tree
  else
    pip3 install --user pipx jinja2 requests
    sudo apt install -y python3-apt python3-venv rsync tree
  fi
  python3 -m pipx install --include-deps ansible
  python3 -m pipx ensurepath
  # TODO: support other shells
  source "${HOME}/.bashrc"
  pushd "${AEM_HOME}" > /dev/null
  # update ansible scripts
  ${HOME}/.local/bin/ansible-galaxy install --force -r galaxy-requirements.yaml

  if [[ ${APOLLO_ENV_USE_GPU_HOST} -eq 1 ]]; then
    if check_nvidia_gpu_available; then
      export USE_GPU_HOST=1
      ${HOME}/.local/bin/ansible-playbook ansible/env-with-cuda.yml
    elif check_amd_gpu_available; then
      # TODO: support ROCm
      # export USE_GPU_HOST=1
      # ${HOME}/.local/bin/ansible-playbook ansible/env-with-rocm.yml
      export USE_GPU_HOST=0
      export APOLLO_ENV_USE_GPU_HOST=0
      ${HOME}/.local/bin/ansible-playbook ansible/env-minimal.yml
    else
      export USE_GPU_HOST=0
      export APOLLO_ENV_USE_GPU_HOST=0
      ${HOME}/.local/bin/ansible-playbook ansible/env-minimal.yml
    fi
  else
    export USE_GPU_HOST=0
    ${HOME}/.local/bin/ansible-playbook ansible/env-minimal.yml
  fi

  popd > /dev/null

  # install buildtool
  # TODO: fetch from remote
  local buildtool_version="10.0.0-beta-r1"
  # dpkg -x "/apollo_workspace/apollo-neo-buildtool_${buildtool_version}_amd64.deb" "${APOLLO_ENV_ROOT}"
  install -d -m 755 "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/${buildtool_version}"
  wget -c "https://apollo-system.cdn.bcebos.com/archive/10.0/buildtool-${buildtool_version}.tar.gz" -O - | tar -zx -C "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/${buildtool_version}" --strip-components=1
  subenv "${AEM_HOME}/buildtool.conf.tpl" "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/${buildtool_version}/config/module.conf"
  ln -snf "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/${buildtool_version}" "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/latest"
  install -d -m 755 "${APOLLO_ENV_ROOT}/opt/apollo/neo/bin"
  ln -snf "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/latest/bin/mainboard.py" "${APOLLO_ENV_ROOT}/opt/apollo/neo/bin/buildtool"

  # install -D -m 755 "${AEM_HOME}/update_dylib.sh" "${APOLLO_ENV_ROOT}/opt/apollo/neo/update_dylib.sh"
  ln -snf "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/latest/scripts/update_dylib.sh" "${APOLLO_ENV_ROOT}/opt/apollo/neo/update_dylib.sh"
  # # overwrite update_dylib.sh in buildtool
  # install -D -m 755 "${AEM_HOME}/update_dylib.sh" "${APOLLO_ENV_ROOT}/opt/apollo/neo/packages/buildtool/latest/scripts/update_dylib.sh"

  # create bazel-extended-tools symbol link
  install -d -m 755 "${APOLLO_ENV_ROOT}/opt/apollo/neo/src"
  ln -snf "/opt/apollo/neo/packages/bazel-extend-tools/latest/src" "${APOLLO_ENV_ROOT}/opt/apollo/neo/src/tools"

  touch "${APOLLO_ENV_HOME}/inited"

  apollo_save_envconfig
}
export -f apollo_create_hostenv

apollo_exec_env() {
  if [[ "${APOLLO_ENV_BACKEND}" == "docker" ]]; then
    apollo_exec_container "$@"
  elif [[ "${APOLLO_ENV_BACKEND}" == "host" ]]; then
    apollo_exec_hostenv "$@"
  else
    fatal "unsupported backend: ${APOLLO_ENV_BACKEND}"
    exit 1
  fi
}
export -f apollo_exec_env

apollo_exec_hostenv() {
  local cmd="${1}"
  shift
  if [[ -z "${cmd}" ]]; then
    fatal "no command specified"
    exit 1
  fi
  /bin/bash --rcfile <(
    cat "${AEM_HOME}/activate.sh"
    echo 'PS1="('${APOLLO_ENV_NAME}') $PS1"'
  ) -c "${cmd} $*"
}
export -f apollo_exec_hostenv

apollo_exec_container() {
  local cmd="${1}"
  shift
  if [[ -z "${cmd}" ]]; then
    fatal "no command specified"
    exit 1
  fi
  local envs=()
  for x in ${!APOLLO_ENV_@}; do
    envs+=('-e' "${x}=${!x}")
  done
  # add command line history file
  envs+=('-e' "HISTFILE=${APOLLO_ENV_WORKROOT}/.cache/.bash_history")
  envs+=('-e' 'HISTSIZE=200000')
  envs+=('-e' 'HISTFILESIZE=200000')
  envs+=('-e' 'HISTCONTROL=ignorespace:erasedups')

  # compatibility with old version
  envs+=('-e' "DOCKER_USER=${APOLLO_ENV_CONTAINER_USER}")

  # Allow X server connection from container.
  xhost +local:root 1> /dev/null 2>&1

  docker exec \
    -u "${APOLLO_ENV_CONTAINER_USER}" \
    "${envs[@]}" \
    -w "${APOLLO_ENV_WORKROOT}" \
    -it \
    "${APOLLO_ENV_CONTAINER_NAME}" \
    /bin/bash -c "${cmd} $*"

  xhost -local:root 1> /dev/null 2>&1
}
export -f apollo_exec_container

apollo_enter_env() {
  if [[ "${APOLLO_ENV_BACKEND}" == "docker" ]]; then
    apollo_enter_container
  elif [[ "${APOLLO_ENV_BACKEND}" == "host" ]]; then
    apollo_enter_hostenv
  else
    fatal "unsupported backend: ${APOLLO_ENV_BACKEND}"
    exit 1
  fi
}
export -f apollo_enter_env

apollo_enter_hostenv() {
  /bin/bash --rcfile <(
    cat "${AEM_HOME}/activate.sh"
    echo 'PS1="('${APOLLO_ENV_NAME}') $PS1"'
  ) -i
}
export -f apollo_enter_hostenv

apollo_determine_image() {
  local image="${APOLLO_ENV_CONTAINER_IMAGE}"
  if [[ -z "${image}" ]]; then
    local repo="${APOLLO_ENV_CONTAINER_REPO:-${DOCKER_REPO}}"
    local tag="${APOLLO_ENV_CONTAINER_TAG:-${VERSION}}"
    local target_arch="$(uname -m)"
    if [[ -n "${APOLLO_ENV_CROSS_PLATFORM}" ]]; then
      target_arch="${APOLLO_ENV_CROSS_PLATFORM}"
    fi
    if [[ -z "${repo}" ]]; then
      if [[ "${target_arch}" == "aarch64" ]]; then
        repo="${APOLLO_ENV_CONTAINER_REPO_ARM}"
      elif [[ "${target_arch}" == "x86_64" ]]; then
        repo="${APOLLO_ENV_CONTAINER_REPO_X86}"
      else
        fatal "unsupported target architecture: ${target_arch}"
        exit 1
      fi
    fi
    if [[ -z "${tag}" ]]; then
      if [[ "${target_arch}" == "aarch64" ]]; then
        tag="${APOLLO_ENV_CONTAINER_TAG_ARM}"
      elif [[ "${target_arch}" == "x86_64" ]]; then
        tag="${APOLLO_ENV_CONTAINER_TAG_X86}"
      else
        fatal "unsupported target architecture: ${target_arch}"
        exit 1
      fi
      tag="${tag:-latest}"
    fi
    image="${repo}:${tag}"
  fi
  export APOLLO_ENV_CONTAINER_IMAGE="${image}"
  echo "${image}"
}
export -f apollo_determine_image

apollo_create_container_gpu_options() {
  local gpu_opts=()
  if [[ "${AEM_FLAG_USE_GPU}" == "1" ]]; then
    if check_nvidia_gpu_available; then
      if command -v nvidia-container-toolkit &> /dev/null; then
        local docker_version
        docker_version="$(docker version --format '{{.Server.Version}}')"
        if [[ "$(cmp_version "${docker_version}" "19.03")" -ge 0 ]]; then
          if [[ "$(uname -m)" == "aarch64" ]]; then
            gpu_opts+=('--runtime' 'nvidia')
          else
            gpu_opts+=('--gpus' 'all')
          fi
          export USE_GPU_HOST=1
          export APOLLO_ENV_USE_GPU_HOST=${USE_GPU_HOST}
        else
          warn "please upgrade docker to 19.03+ to access GPU from container"
          export USE_GPU_HOST=0
        fi
      elif command -v nvidia-container-runtime &> /dev/null; then
        gpu_opts+=('--runtime' 'nvidia')
        export USE_GPU_HOST=1
      elif command -v nvidia-docker &> /dev/null; then
        warn "nvidia-docker is deprecated, please use nvidia-container-toolkit or nvidia-container-runtime"
        export USE_GPU_HOST=0
      else
        warn "no nvidia-container-toolkit or nvidia-container-runtime found, use CPU fallback."
        export USE_GPU_HOST=0
      fi
    elif check_amd_gpu_available; then
      gpu_opts+=('--device=/dev/kfd' '--device=/dev/dri' '--security-opt' 'seccomp=unconfined' '--group-add' 'video')
      export USE_GPU_HOST=1
    else
      warn "no GPU device available, use CPU fallback."
      export USE_GPU_HOST=0
    fi
  fi
  # sync to new environment
  export APOLLO_ENV_USE_GPU_HOST=${USE_GPU_HOST}

  echo "${gpu_opts[*]}"
}
export -f apollo_create_container_gpu_options

apollo_create_container_volume_options() {
  local volume_opts=()

  # system devices
  volume_opts+=('-v' '/dev:/dev')
  # shared directories
  volume_opts+=('-v' '/media:/media')
  # X
  volume_opts+=('-v' '/tmp/.X11-unix:/tmp/.X11-unix:rw')
  # kernel modules
  volume_opts+=('-v' '/lib/modules:/lib/modules')

  # auca
  auca_sdk_so="/usr/lib/libapollo-auca-sdk.so.1"
  if [[ -x ${auca_sdk_so} ]]; then
    volume_opts+=('-v' "${auca_sdk_so}:${auca_sdk_so}")
  fi

  # tegrastats
  tegrastats="/usr/bin/tegrastats"
  if [[ -x ${tegrastats} ]]; then
    volume_opts+=('-v' "${tegrastats}:${tegrastats}")
  fi

  # arm igpu kernel data
  [[ $(uname -m) == aarch64 ]] && [[ -e "/sys/kernel/debug" ]] &&
    volume_opts+=('-v' '/sys/kernel/debug:/sys/kernel/debug')

  volume_opts+=('-v' '/dev/null:/dev/raw1394')

  # volume for apollo packages and configurations
  for x in {apollo,opt}; do
    local volume_name="${APOLLO_ENV_CONTAINER_PREFIX}${APOLLO_ENV_NAME}_${x}"
    volume_opts+=('-v' "${volume_name}:/${x}")
  done

  # volume of user shared configurations' and resources' directories
  volume_opts+=('-v' "${HOME}/.apollo:/home/${APOLLO_ENV_CONTAINER_USER}/.apollo")

  # custom volumes
  for x in "${APOLLO_ENV_MOUNTS[@]}"; do
    mount_dst="${x#*:}"
    mount_opt=''
    if [[ "${mount_dst}" =~ .*:.* ]]; then
      mount_opt="${mount_dst#*:}"
      mount_dst="${mount_dst%:*}"
    fi
    mount_src="${x%%:*}"
    if [[ -n "${mount_opt}" ]]; then
      volume_opts+=('-v' "${mount_src}:${mount_dst}:${mount_opt}")
    else
      volume_opts+=('-v' "${mount_src}:${mount_dst}")
    fi
    declare "flag_${mount_dst//\//_}"=1
  done

  # workspace, only available in docker backend
  mount_dst="${APOLLO_ENV_WORKROOT}"
  customized="flag_${mount_dst//\//_}"
  if [[ "${!customized}" != "1" ]]; then
    volume_opts+=('-v' "${APOLLO_ENV_WORKSPACE}:${mount_dst}")
  fi

  # fix hardcode resource and conf paths under workspace
  # TODO: will be removed in the future
  fix_mounts=(
    "${APOLLO_ENV_WORKSPACE}/data:/apollo/data"
    "${APOLLO_ENV_WORKSPACE}/data:/opt/apollo/neo/data"
    "${APOLLO_ENV_WORKSPACE}/data/log:/opt/apollo/neo/data/log"
    "${APOLLO_ENV_WORKSPACE}/data/log:/apollo/data/log"
    "${APOLLO_ENV_WORKSPACE}/output:/apollo/output"
    "${APOLLO_ENV_WORKSPACE}/log:/apollo/log"
    "${APOLLO_ENV_WORKSPACE}/data/calibration_data:/apollo/modules/calibration/data"
    "${APOLLO_ENV_WORKSPACE}/data/map_data:/apollo/modules/map/data"
  )
  for x in "${fix_mounts[@]}"; do
    mount_src="${x%%:*}"
    mount_dst="${x#*:}"
    customized="flag_${mount_dst//\//_}"
    # ensure host directory exists
    mkdir -p "${mount_src}"
    # if not customized, add volume
    if [[ "${!customized}" != "1" ]]; then
      volume_opts+=('-v' "${mount_src}:${mount_dst}")
    fi
  done

  echo "${volume_opts[*]}"
}
export -f apollo_create_container_volume_options

apollo_execute_cmd_in_container() {
  ${DOCKER} exec -u root "${APOLLO_ENV_CONTAINER_NAME}" bash -c "$*"
}
export -f apollo_execute_cmd_in_container

apollo_container_execute_cmd() {
  "${DOCKER}" exec -u "${APOLLO_ENV_CONTAINER_USER}" "${APOLLO_ENV_CONTAINER_NAME}" bash -c "$*"
}
export -f apollo_container_execute_cmd

apollo_container_created_start_user() {
  user="${SUDO_USER-$USER}"
  container_aem_path="/opt/apollo/aem"

  if [ "${user}" != "root" ]; then
    apollo_execute_cmd_in_container "bash -c ${container_aem_path}/docker_start_user.sh"
  fi
}
export -f apollo_container_created_start_user

apollo_container_download_arm_lib() {
  if [[ -n "${APOLLO_ENV_CROSS_PLATFORM}" ]]; then
    if [[ $(uname -m) == "x86_64" ]]; then
      if [[ "${APOLLO_ENV_CROSS_PLATFORM}" == "aarch64" ]]; then
        info "download external library for cross-compilation..."
        local tegra_lib_url="https://apollo-pkg-beta.cdn.bcebos.com/archive/tegra.tar.gz"
        apollo_execute_cmd_in_container "cd ~ && wget -nv ${tegra_lib_url} && \
                  tar -xzvf ~/tegra.tar.gz -C /usr/lib/aarch64-linux-gnu/ > /dev/null"
      fi
    fi
  fi
}
export -f apollo_container_download_arm_lib

apollo_container_created_post_action() {
  local init_packages=(
    'apollo-neo-buildtool'
  )
  aem_path="${AEM_HOME}"
  container_aem_path="/opt/apollo/aem"

  ${DOCKER} cp "${aem_path}" "${APOLLO_ENV_CONTAINER_NAME}":"${container_aem_path}"
  apollo_execute_cmd_in_container "ln -snf ${container_aem_path}/aem /usr/bin/aem"
  apollo_execute_cmd_in_container "ln -snf ${container_aem_path}/auto_complete.bash /etc/bash_completion.d/aem"
  apollo_execute_cmd_in_container "ln -snf ${container_aem_path}/auto_complete.zsh /usr/share/zsh/functions/Completion/Unix/_aem"
  apollo_execute_cmd_in_container "[[ $(uname -m) == "aarch64" ]] && [[ -e /sys/kernel/debug ]] && chmod +rx /sys/kernel/debug"
  apollo_execute_cmd_in_container "apt update && apt install --only-upgrade -y ${init_packages[@]}"
  apollo_execute_cmd_in_container "chmod 777 /opt/apollo/neo/packages/buildtool/latest/setup.sh"
  apollo_execute_cmd_in_container "mkdir -pv /opt/apollo/neo/etc && chmod 777 -R /opt/apollo/neo/etc"
  apollo_container_created_start_user
  apollo_execute_cmd_in_container "mkdir -pv /apollo_workspace/data/{log,bag,record} &&
          chown -R ${APOLLO_ENV_CONTAINER_USER}:${APOLLO_ENV_CONTAINER_GROUP} /apollo_workspace/data/"
  apollo_container_execute_cmd "buildtool -v"
  # TODO: migrate to active script like host env
  apollo_container_execute_cmd "echo [[ -e /opt/apollo/neo/setup.sh ]] \&\& source /opt/apollo/neo/setup.sh >> /home/${APOLLO_ENV_CONTAINER_USER}/.bashrc"
  apollo_container_download_arm_lib
}
export -f apollo_container_created_post_action

apollo_create_container_env_options() {
  local env_opts=()
  for x in ${!APOLLO_ENV_@}; do
    env_opts+=('-e' "${x}=${!x}")
  done

  # GPU environment
  env_opts+=('-e' "USE_GPU_HOST=${USE_GPU_HOST}")
  if check_nvidia_gpu_available; then
    env_opts+=('-e' 'NVIDIA_VISIBLE_DEVICES=all')
    env_opts+=('-e' 'NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility')
  fi

  # X
  env_opts+=('-e' "DISPLAY=${DISPLAY}")

  # user info and container info
  env_opts+=('-e' "USER=${APOLLO_ENV_CONTAINER_USER}")
  env_opts+=('-e' "DOCKER_USER=${APOLLO_ENV_CONTAINER_USER}")
  env_opts+=('-e' "DOCKER_USER_ID=${APOLLO_ENV_CONTAINER_UID}")
  env_opts+=('-e' "DOCKER_GRP=${APOLLO_ENV_CONTAINER_GROUP}")
  env_opts+=('-e' "DOCKER_GRP_ID=${APOLLO_ENV_CONTAINER_GID}")
  env_opts+=('-e' "DOCKER_IMG=${APOLLO_ENV_CONTAINER_IMAGE}")

  # cross-platform
  if [[ -n "${APOLLO_ENV_CROSS_PLATFORM}" ]]; then
    env_opts+=('-e' "CROSS_PLATFORM=1")
  else
    env_opts+=('-e' "CROSS_PLATFORM=0")
  fi

  # shell history
  env_opts+=('-e' "HISTFILE=${APOLLO_ENV_WORKROOT}/.cache/.bash_history")

  # eplite
  cat /etc/bash.bashrc | grep AIPE_WITH_UNIX_DOMAIN_SOCKET > /dev/null 2>&1
  [[ $? == 0 ]] && env_opts+=('-e' "AIPE_WITH_UNIX_DOMAIN_SOCKET=ON")

  echo "${env_opts[*]}"
}
export -f apollo_create_container_env_options

apollo_load_envconfig() {
  local env_name="${APOLLO_ENV_NAME}"
  local env_home="${APOLLO_ENVS_ROOT}/${env_name}"

  if [[ -f "${env_home}/env.config" ]]; then
    source "${env_home}/env.config"
  fi
}
export -f apollo_load_envconfig

apollo_save_envconfig() {
  env_name="${APOLLO_ENV_NAME}"
  env_home="${APOLLO_ENVS_ROOT}/${env_name}"

  for x in ${!AEM_@}; do
    declare -p "${x}" | sed -e 's/^declare -[^ ]* /export /' >> "${env_home}/env.config"
  done

  for x in ${!APOLLO_@}; do
    declare -p "${x}" | sed -e 's/^declare -[^ ]* /export /' >> "${env_home}/env.config"
  done

  chmod +x "${env_home}/env.config"
}
export -f apollo_save_envconfig

apollo_create_container() {

  image="$(apollo_determine_image)"
  if ! docker_pull "${image}"; then
    error "failed to pull docker image ${DEV_IMAGE}"
    exit 1
  fi

  options=(
    '-itd'
    '--privileged'
    '--name' "${APOLLO_ENV_CONTAINER_NAME}"
    '--label' "owner=${USER}"
  )

  gpu_opts=($(apollo_create_container_gpu_options))
  options+=("${gpu_opts[@]}")

  volume_opts=($(apollo_create_container_volume_options))
  options+=("${volume_opts[@]}")
  options+=('-w' "${APOLLO_ENV_WORKROOT}")

  env_opts=($(apollo_create_container_env_options))
  # for x in ${!APOLLO_ENV_@}; do
  #   env_opts+=('-e' "${x}=${!x}")
  # done
  options+=("${env_opts[@]}")

  options+=(
    '--pid=host'
    '--network' 'host'
    '--add-host' "$(hostname):127.0.0.1"
    '--add-host' 'in-dev-docker:127.0.0.1'
    '--hostname' 'in-dev-docker'
    '--shm-size' "${AEM_FLAG_SHM_SIZE}"
    '--entrypoint' "${APOLLO_ENV_CONTAINER_SHELL}"
  )

  info "creating container ${APOLLO_ENV_CONTAINER_NAME} from image ${image}..."
  set -x
  ${DOCKER} run "${options[@]}" "${image}" "${APOLLO_ENV_ARGS[@]}"
  set +x
  apollo_container_created_post_action
  apollo_save_envconfig
}
export -f apollo_create_container

apollo_start_stopped_container() {
  if docker_container_stopped "${APOLLO_ENV_CONTAINER_NAME}"; then
    docker start "${APOLLO_ENV_CONTAINER_NAME}" 1> /dev/null
  fi
}
export -f apollo_start_stopped_container

apollo_enter_container() {
  apollo_start_stopped_container

  local envs=()
  for x in ${!APOLLO_ENV_*}; do
    envs+=('-e' "${x}=${!x}")
  done
  # add command line history file
  envs+=('-e' "HISTFILE=${APOLLO_ENV_WORKROOT}/.cache/.bash_history")
  envs+=('-e' 'HISTSIZE=200000')
  envs+=('-e' 'HISTFILESIZE=200000')
  envs+=('-e' 'HISTCONTROL=ignorespace:erasedups')

  # compatibility with old version
  envs+=('-e' "DOCKER_USER=${APOLLO_ENV_CONTAINER_USER}")

  # Allow X server connection from container.
  xhost +local:root 1> /dev/null 2>&1

  # TODO: support custom shell
  docker exec \
    -u "${APOLLO_ENV_CONTAINER_USER}" \
    ${envs[@]} \
    -w "${APOLLO_ENV_WORKROOT}" \
    -it \
    "${APOLLO_ENV_CONTAINER_NAME}" \
    /bin/bash

  xhost -local:root 1> /dev/null 2>&1
}
export -f apollo_enter_container

apollo_remove_hostenv() {
  :
}
export -f apollo_remove_hostenv

apollo_remove_container() {
  info "removing container ${APOLLO_ENV_CONTAINER_NAME}"
  docker rm -f "${APOLLO_ENV_CONTAINER_NAME}" 1> /dev/null 2>&1
}
export -f apollo_remove_container

apollo_remove_envhome() {
  local env_home_global="${APOLLO_ENVS_ROOT}/${APOLLO_ENV_NAME}"
  local env_home_local="${APOLLO_ENV_WORKSPACE}/.aem"

  if [[ "${APOLLO_ENV_BACKEND}" == "docker" ]]; then
    container_name="${APOLLO_ENV_CONTAINER_PREFIX}${APOLLO_ENV_NAME}"
    for x in {apollo,opt}; do
      local volume_name="${container_name}_${x}"
      if docker_volume_exists "${volume_name}"; then
        volume_dev="$(docker volume inspect ${volume_name} --format '{{.Options.device}}')"
        info "removing volume ${volume_name} ${volume_dev}"
        docker volume rm -f "${volume_name}" 1> /dev/null 2>&1
        if [[ -n "${volume_dev}" ]]; then
          sudo rm -rf "${volume_dev}"
        fi
      fi
    done
  fi

  sudo rm -rf "${env_home_global}"
  sudo rm -rf "${env_home_local}"
}
export -f apollo_remove_envhome

apollo_remove_env_old() {
  local container_name="${APOLLO_ENV_CONTAINER_PREFIX}${APOLLO_ENV_NAME}"
  local envhome="${APOLLO_ENVS_ROOT}/${container_name}"
  local worklocal=0

  if docker_container_exists "${container_name}"; then
    container_envs="$(docker inspect ${container_name} --format='{{join .Config.Env "\n"}}' | sed -e 's:=\(.*\):="\1":')"
    workspace=$(source <(echo -e "${container_envs}") && echo "${APOLLO_ENV_WORKSPACE}")
    worklocal=$(source <(echo -e "${container_envs}") && echo "${APOLLO_ENV_WORKLOCAL}")
  else
    warn "container ${container_name} not found, maybe it was removed manually, try cleaning up"
    for x in apollo opt; do
      local vol="${container_name}_${x}"
      if docker_volume_exists "${vol}"; then
        local vol_dev="$(docker volume inspect "${vol}" --format '{{.Options.device}}')"
        if [[ -n "${vol_dev}" ]]; then
          envroot="$(dirname ${vol_dev})"
          envhome="$(dirname ${envroot})"
          break
        fi
      fi
      if [[ "${envhome}" == ${APOLLO_ENVS_ROOT}* ]]; then
        worklocal=0
      else
        worklocal=1
        workspace="$(dirname ${envhome})"
      fi
    done
  fi

  for x in apollo opt; do
    local vol="${container_name}_${x}"
    if docker_volume_exists "${vol}"; then
      local vol_dev="$(docker volume inspect "${vol}" --format '{{.Options.device}}')"
      if [[ -n "${vol_dev}" ]]; then
        info "removing volume ${vol} ${vol_dev}"
      else
        info "removing volume ${vol}"
      fi
      docker volume rm -f "${vol}" 1> /dev/null 2>&1
      sudo rm -rf "${vol_dev}"
    fi
  done

  info "removing env home ${envhome}"
  sudo rm -rf "${envhome}"

  if [[ "${worklocal}" == "0" ]]; then
    workspace=${workspace:-${APOLLO_ENV_WORKSPACE}}
    if [[ -n "${workspace}" ]]; then
      if [[ -L "${workspace}/.aem/envroot" ]]; then
        if [[ ! -e "${workspace}/.aem/envroot" ]]; then
          # deadlink, remove it
          rm -f "${workspace}/.aem/envroot"
        elif [[ "$(readlink ${workspace}/.aem/envroot)" == "${envroot}" ]]; then
          rm -f "${workspace}/.aem/envroot"
        fi
      fi
    fi
  fi
}
export -f apollo_remove_env_old
