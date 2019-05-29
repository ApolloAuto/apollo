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

mkdir -p proto_bundle

# proto dependencies
COMMON_PROTOS='../../common/proto/*.proto ../../common/configs/proto/vehicle_config.proto'
LOCALIZATION_PROTOS='../../localization/proto/localization.proto ../../localization/proto/pose.proto ../../localization/proto/localization_status.proto'
CHASSIS_PROTOS='../../canbus/proto/chassis.proto'
PLANNING_PROTOS='../../planning/proto/*.proto'
PREDICTION_PROTOS='../../prediction/proto/feature.proto ../../prediction/proto/lane_graph.proto ../../prediction/proto/prediction_point.proto'
PERCEPTION_PROTOS='../../perception/proto/traffic_light_detection.proto ../../perception/proto/perception_obstacle.proto'
REALTIVE_MAP_PROTOS='../../map/relative_map/proto/*.proto'
MAP_PROTOS='../../map/proto/*.proto'
MONITOR_PROTOS='../../common/monitor_log/proto/monitor_log.proto'
ROUTING_PROTOS='../../routing/proto/routing.proto'

node_modules/protobufjs/bin/pbjs -t json ../proto/simulation_world.proto ../proto/chart.proto \
    $COMMON_PROTOS $LOCALIZATION_PROTOS $CHASSIS_PROTOS $PLANNING_PROTOS \
    $PERCEPTION_PROTOS $MONITOR_PROTOS $ROUTING_PROTOS $MAP_PROTOS \
    $PREDICTION_PROTOS $REALTIVE_MAP_PROTOS \
    -o proto_bundle/sim_world_proto_bundle.json

node_modules/protobufjs/bin/pbjs -t json ../proto/point_cloud.proto \
    -o proto_bundle/point_cloud_proto_bundle.json
