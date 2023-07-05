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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
APOLLO_OUTPUT="/opt/apollo/neo/packages"
clear_dev_scripts=${TOP_DIR}/scripts/clear.py

dreamview_launch=${APOLLO_OUTPUT}/dreamview-dev/local/launch/dreamview.launch
dreamview_src_launch=${APOLLO_OUTPUT}/dreamview-dev/local/src/launch/dreamview.launch
planning_dag=${APOLLO_OUTPUT}/planning-dev/local/dag/planning.dag
planning_src_dag=${APOLLO_OUTPUT}/planning-dev/local/src/dag/planning.dag

sed -i "s/\/apollo\/bazel-bin\/modules\/dreamview\/dreamview/dreamview/g" $dreamview_launch
sed -i "s/\/apollo\/bazel-bin\/modules\/dreamview\/dreamview/dreamview/g" $dreamview_src_launch

sed -i "s/\/opt\/apollo\/neo\/packages\/planning-gpu-dev\/latest\/lib\/libplanning_component.so/\/opt\/apollo\/neo\/packages\/planning-dev\/latest\/lib\/libplanning_component.so/g" $planning_dag
sed -i "s/\/opt\/apollo\/neo\/packages\/planning-gpu-dev\/latest\/lib\/libplanning_component.so/\/opt\/apollo\/neo\/packages\/planning-dev\/latest\/lib\/libplanning_component.so/g" $planning_src_dag

python3 $clear_dev_scripts

