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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd  -P)"
if [[ -f "/opt/apollo/neo/packages/tools-dev/latest/vehicle_calibration/result2pb" ]]; then
    /opt/apollo/neo/packages/tools-dev/latest/vehicle_calibration/result2pb /opt/apollo/neo/packages/control-dev/latest/conf/control_conf.pb.txt $1
else
    ${TOP_DIR}/bazel-bin/modules/tools/vehicle_calibration/result2pb ${TOP_DIR}/modules/control/conf/control_conf.pb.txt $1
fi

echo "Created control conf file: control_conf_pb.txt"

