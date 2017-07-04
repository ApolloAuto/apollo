#!/usr/bin/env python

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

from modules.canbus.proto import chassis_detail_pb2
from modules.canbus.proto import chassis_pb2
from modules.common.monitor.proto import monitor_pb2
from modules.common.configs.proto import config_extrinsics_pb2
from modules.common.configs.proto import vehicle_config_pb2
from modules.common.proto import geometry_pb2
from modules.common.proto import header_pb2
from modules.control.proto import control_cmd_pb2
from modules.control.proto import pad_msg_pb2
from modules.decision.proto import decision_pb2
from modules.localization.proto import localization_pb2
from modules.localization.proto import gps_pb2
from modules.localization.proto import imu_pb2
from modules.perception.proto import perception_obstacle_pb2
from modules.perception.proto import traffic_light_detection_pb2
from modules.planning.proto import planning_internal_pb2
from modules.planning.proto import planning_pb2
from modules.prediction.proto import prediction_obstacle_pb2
