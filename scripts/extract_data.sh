#! /usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

TARGET_DIR="${TOP_DIR}/sensor_calibration/lidar_to_gnss"
RECORD_FILES=()
RECORD_DIRS=()

function print_usage() {
  echo 'Usage:
    ./extract_data.sh -f <path/to/record/file> -d <path/to/record/dir>
  eg.
    ./extract_data.sh -f xxx/yyy.record.00000 -f xxx/yyy.record.00001
  or
    ./extract_data.sh -d xxx
  '
}

function parse_args() {
  # read options
  while getopts ':f:d:' flag; do
    case "${flag}" in
      f)
        RECORD_FILES+=("${OPTARG}")
        ;;
      d)
        RECORD_DIRS+=("${OPTARG}")
        ;;
      *)
        print_usage
        exit 1
        ;;
    esac
  done
}

function _check_target_dir() {
  if [ ! -d "${TARGET_DIR}" ]; then
    mkdir -p ${TARGET_DIR}
    cp -R ${TOP_DIR}/docs/Apollo_Fuel/examples/sensor_calibration/lidar_to_gnss/* ${TARGET_DIR}
  elif [[ ! -z "$(ls -A ${TARGET_DIR})" ]]; then
    local warn="The ${TARGET_DIR} is not empty, do you want to delete it? (Y/n)"
    echo "${warn}"
    local answer
    typeset -l answer
    read answer
    if [ "${answer}" == "y" ];then
      rm -rf ${TARGET_DIR}/*
      cp -R ${TOP_DIR}/docs/Apollo_Fuel/examples/sensor_calibration/lidar_to_gnss/* ${TARGET_DIR}
    fi
  else
    cp -R ${TOP_DIR}/docs/Apollo_Fuel/examples/sensor_calibration/lidar_to_gnss/* ${TARGET_DIR}
  fi
}

# Since pypcd installed via `pip install` only works with python2.7,
# we can only install it this way
function _install_pypcd() {
  git clone https://github.com/dimatura/pypcd --depth=1
  pushd pypcd > /dev/null
  git fetch origin pull/9/head:python3 && git checkout python3
  python3 setup.py install --user
  popd > /dev/null
  rm -rf pypcd
}

function get_records() {
  for file in "${RECORD_FILES[@]}"; do
    if [ -f "${file}" ]; then
      cp "${file}" "${TARGET_DIR}"
    else
      echo "File ${file} doesn't exist!"
      exit 1
    fi
  done

  for dir in "${RECORD_DIRS[@]}"; do
    if [ -d "${dir}" ]; then
      case "${dir}" in
        */)
          cp ${dir}* "${TARGET_DIR}/records"
          ;;
        *)
          cp ${dir}/* "${TARGET_DIR}/records"
          ;;
      esac
    else
      echo "Directory ${dir} doesn't exist!"
      exit 1
    fi
  done
}

function install_if_not_exist() {
  while [ $# -gt 0 ]; do
    local pkg=$1
    shift
    pip show --files "${pkg}" >/dev/null
    if [ $? -ne 0 ]; then
      if [[ "${pkg}" == "pypcd" ]]; then
        _install_pypcd
      else
        sudo pip install --no-cache-dir "${pkg}"
      fi
    fi
  done
}

function main() {
  if [ "$#" -eq 0 ]; then
    print_usage
    exit 1
  fi

  _check_target_dir

  parse_args "$@"

  if [[ ${#RECORD_FILES[*]} -eq 0 && ${#RECORD_DIRS[*]} -eq 0 ]]; then
    print_usage
    exit 1
  fi

  get_records

  install_if_not_exist "pyyaml" "pypcd"

  bazel run //modules/tools/sensor_calibration:extract_data \
    -- --config ${TARGET_DIR}/lidar_to_gnss.config

  rm -f ${TARGET_DIR}/records/*
}

main "$@"
