#!/usr/bin/env python3

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
###############################################################################
"""
Lidar Template
"""

LIDDAR_CONF_BASE_TPL = """
config_base {{
  scan_channel: "{scan_channel}"
  point_cloud_channel: "{point_cloud_channel}"
  frame_id: "{frame_id}"
  # sample ONLINE_LIDAR, RAW_PACKET
  source_type: {source_type}
  write_scan: {write_scan}
}}
"""

RSLIDAR_CONF_TPL = LIDDAR_CONF_BASE_TPL + """
# sample: "RS16", "RS32", "RS80", "RS128", "RSBP", "RSHELIOS", "RSHELIOS_16P", "RSM1"
model: "{model}"
msop_port: {msop_port}
difop_port: {difop_port}
echo_mode: {echo_mode}
start_angle: {start_angle}
end_angle: {end_angle}
min_distance: {min_distance}
max_distance: {max_distance}
cut_angle: {cut_angle}
split_frame_node: {split_frame_node}
num_pkts_split: {num_pkts_split}
use_lidar_clock: {use_lidar_clock}
dense_points: {dense_points}
ts_first_point: {ts_first_point}
wait_for_difop: {wait_for_difop}
config_from_file: {config_from_file}
angle_path: "{angle_path}"
split_angle: {split_angle}
send_raw_packet: {send_raw_packet}
"""

LIVOX_CONF_TPL = LIDDAR_CONF_BASE_TPL + """
lidar_config_file_path: "{config_file_path}"
enable_sdk_console_log: {enable_sdk_console_log}
integral_time: {integral_time}
use_lidar_clock: {use_lidar_clock}
lidar_ip: "{lidar_ip}"
{custom_integral}
"""

VANJEE_CONF_TPL = LIDDAR_CONF_BASE_TPL + """
# sample: "vanjee_720_16,vanjee_720_32"
model: "{model}"
connect_type: {connect_type}
host_msop_port: {host_msop_port}
lidar_msop_port: {lidar_msop_port}
host_address: "{host_address}"
lidar_address: "{lidar_address}"
publish_mode: {publish_mode}
start_angle: {start_angle}
end_angle: {end_angle}
min_distance: {min_distance}
max_distance: {max_distance}
use_lidar_clock: {use_lidar_clock}
dense_points: {dense_points}
wait_for_difop: {wait_for_difop}
config_from_file: {config_from_file}
angle_path: "{angle_path}"
"""

LIDAR_FUSION_AND_COMPENSATOR_CONF_TPL = """
max_interval_ms: {max_interval_ms}
drop_expired_data : {drop_expired_data}
input_channel: [
{input_channels}
]
# wait time after main channel receive msg, unit second
wait_time_s: {wait_time_s}
world_frame_id: "world"
transform_query_timeout: {transform_query_timeout}
output_channel: "/apollo/sensor/lidar/compensator/PointCloud2"
rotation_compensation: {rotation_compensation}
target_frame_id: "{target_frame_id}"

{filter_configs}
"""

LIDAR_COMPONENT_TPL = """
    components {{
        class_name : "{class_name}"
        config {{
            name : "{config_name}"
            config_file_path : "{config_path}"
        }}
    }}
"""

LIDAR_DRIVER_DAG_TPL = """
module_config {{
    module_library : "{lidar_component_lib}"

    ##################################################
    #                   drivers                      #
    ##################################################
    {components}
}}
"""

LIDAR_FUSION_AND_COMPENSATOR_DAG_TPL = """
##################################################
#                   fusion and compensation      #
##################################################
module_config {{
    module_library : "modules/drivers/lidar_fusion_and_compensator/liblidar_fusion_and_compensator_component.so"
    components {{
        class_name : "FusionAndCompensatorComponent"
        config {{
            name : "fusion_and_compensator"
            config_file_path : "{config_file_path}"
            readers {{
                channel: "{read_channel}"
            }}
        }}
    }}
}}
"""
