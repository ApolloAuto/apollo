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

TARGET_DIR="/apollo/sensor_calibration/lidar_to_gnss/records/"
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
  if [[ ! -z $(ls -A ${TARGET_DIR} | grep -v "readme.txt") ]]; then
    local warn="${TARGET_DIR} is not empty, do you want to delete the existing records? (Y/n)"
    echo "${warn}"
    local answer
    typeset -l answer
    read -n 1 answer
    if [ "${answer}" == "y" ];then
      rm -rf ${TARGET_DIR}*
    fi
  fi
}

function get_records() {
  _check_target_dir

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
          cp ${dir}* "${TARGET_DIR}"
          ;;
        *)
          cp ${dir}/* "${TARGET_DIR}"
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
      sudo pip install --no-cache-dir "${pkg}"
    fi
  done
}

function main() {
  if [ "$#" -eq 0 ]; then
    print_usage
    exit 1
  fi

  if [ ! -d "${TOP_DIR}/sensor_calibration" ]; then
    cp -r ${TOP_DIR}/docs/Apollo_Fuel/examples/sensor_calibration ${TOP_DIR}/sensor_calibration
  fi

  parse_args "$@"

  if [[ ${#RECORD_FILES[*]} -eq 0 && ${#RECORD_DIRS[*]} -eq 0 ]]; then
    print_usage
    exit 1
  fi

  get_records

  install_if_not_exist "pyyaml" "pypcd"

  bazel run //modules/tools/sensor_calibration:extract_data \
    -- --config /apollo/sensor_calibration/lidar_to_gnss/lidar_to_gnss.config
}

main "$@"
