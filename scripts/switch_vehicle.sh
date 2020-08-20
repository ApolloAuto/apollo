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

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

# Usage:
#   scripts/switch_vehicle.sh <vehicle_data_path>
#
# <vehicle_data_path> is a directory containing vehicle specified data.
# E.g.: modules/calibration/data/mkz8

VEHICLE_PATH=$1
if [ -d ${VEHICLE_PATH} ]; then
  ${APOLLO_BIN_PREFIX}/modules/dreamview/backend/hmi/vehicle_manager_main \
    --vehicle_data_path="${VEHICLE_PATH}"
else
  error "Cannot open directory: ${VEHICLE_PATH}"
  info "Available vehicles:"
  find modules/calibration/data -maxdepth 1 -mindepth 1 -type d
fi
