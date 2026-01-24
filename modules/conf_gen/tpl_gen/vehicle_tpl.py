#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Vehicle Template
"""

CANBUS_CONF_TPL = """
vehicle_parameter {{
  brand: {brand}
  max_enable_fail_attempt: {max_enable_fail_attempt}
  driving_mode: COMPLETE_AUTO_DRIVE
}}

can_card_parameter {{
  brand: SOCKET_CAN_RAW
  type: PCI_CARD
  channel_id: {channel_id}
  interface: NATIVE
}}

enable_debug_mode: false
enable_receiver_log: false
enable_sender_log: false
"""

CANBUS_FLAGS_TPL = """
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--canbus_conf_file=/apollo/modules/canbus/conf/canbus_conf.pb.txt
--load_vehicle_library=/opt/apollo/neo/lib/modules/canbus_vehicle/{canbus_vehicle_lib}
--load_vehicle_class_name={canbus_vehicle_class}
--enable_chassis_detail_pub
--enable_chassis_detail_sender_pub
--chassis_debug_mode=false
--pad_msg_delay_interval=3
--control_period=0.01
--max_control_miss_num=10
--guardian_period=0.01
--max_guardian_miss_num=10
--use_control_cmd_check=true
--use_guardian_cmd_check=true
--receive_guardian
--estop_brake=30.0
"""

VEHICLE_PARAMS_CONF_TPL = """
vehicle_param {{
  # 车型名称
  brand: {brand}
  vehicle_id {{
      other_unique_id: "{vehicle_id}"        # 车型id
  }}
  front_edge_to_center: {front_edge_to_center}    # 后轴中心到车辆前边缘距离
  back_edge_to_center: {back_edge_to_center}    # 后轴中心到车辆后边缘距离
  left_edge_to_center: {left_edge_to_center}    # 后轴中心到车辆左边缘距离
  right_edge_to_center: {right_edge_to_center}    # 后轴中心到车辆右边缘距离
 
  length: {length}    # 整车长度
  width: {width}    # 整车宽度
  height: {height}   # 整车高度
  min_turn_radius: {min_turn_radius}    # 车辆最小转弯半径
  max_acceleration: {max_acceleration}    # 车辆最大加速度
  max_deceleration: {max_deceleration}    # 车辆最大减速度
  max_steer_angle: {max_steer_angle}    # 车辆方向盘最大转角（弧度）
  max_steer_angle_rate: {max_steer_angle_rate}    # 车辆方向盘最大转速率（弧度/秒）
  steer_ratio: {steer_ratio}    # 车辆方向盘到车前轮转角转换系数
  wheel_base: {wheel_base}    # 车辆前后轴距离
  wheel_rolling_radius: {wheel_rolling_radius}    # 车辆轮胎最小滚动半径
  max_abs_speed_when_stopped: {max_abs_speed_when_stopped}    # 车辆停车会最大绝对速度
  brake_deadzone: {brake_deadzone}    # 车辆刹车踏板无效区
  throttle_deadzone: {throttle_deadzone}    # 车辆油门踏板无效区
}}
"""

LATENCY_PARAM_TPL = """
  {name}_latency_param
  {{
      dead_time: {dead_time}
      rise_time: {rise_time}
      peak_time: {peak_time}
      settling_time: {settling_time}
  }}
"""

CONTROLLER_CONF_TPL = """
controller {{
  name: "{name}"
  type: "{type}"
}}
"""
