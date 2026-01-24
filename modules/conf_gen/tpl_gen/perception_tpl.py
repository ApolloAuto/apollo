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
Perception Template
"""

POINTCLOUD_PREPROCESS_CONF_TPL = """
sensor_name: "{frame_id}"
lidar_query_tf_offset: 20
output_channel_name: "/perception/lidar/pointcloud_preprocess"
plugin_param {{
  name: "PointCloudPreprocessor"
  config_path: "perception/pointcloud_preprocess/data"
  config_file: "pointcloud_preprocessor.pb.txt"
}}
"""

POINTCLOUD_PREPROCESS_DAG_TPL = """
module_config {{
  module_library : "modules/perception/pointcloud_preprocess/libpointcloud_preprocess_component.so"
  components {{
    class_name : "PointCloudPreprocessComponent"
    config {{
      name : "PointCloudPreprocess"
      config_file_path : "/apollo/modules/perception/pointcloud_preprocess/conf/pointcloud_preprocess_config.pb.txt"
      flag_file_path: "/apollo/modules/perception/data/flag/perception_common.flag"
      readers {{
        channel: "{point_cloud_channel}"
      }}
    }}
  }}
}}
"""

POINRTCLOUD_PREPROCESS_FILTER_TPL = """
filter_naninf_points: true
filter_nearby_box_points: true
box_forward_x: {box_forward_x}
box_backward_x: {box_backward_x}
box_forward_y: {box_forward_y}
box_backward_y: {box_backward_y}
filter_high_z_points: true
z_threshold: {z_threshold}
"""

HDMAP_ROI_FILTER_CONF_TPL = """
range: {range}
cell_size: {cell_size}
extend_dist: {extend_dist}
no_edge_table: {no_edge_table}
set_roi_service: {set_roi_service}
"""

LIDAR_SENDOR_META_CONF_TPL = """
sensor_meta {{
    name: "{frame_id}"
    type: VELODYNE_16
    orientation: PANORAMIC
    is_main_sensor: true
}}
"""

CAMERA_SENDOR_META_CONF_TPL = """
sensor_meta {{
    name: "{frame_id}"
    type: {type}
    orientation: {orientation}
}}
"""

TRAFFIC_LIGHT_CONF_TPL = """
tl_tf2_frame_id : "world"
tl_tf2_child_frame_id : "novatel"
tf2_timeout_second : 0.01
camera_names : "{camera_names}"
camera_channel_names : "{camera_channel_names}"
tl_image_timestamp_offset : 0.0
max_process_image_fps : 8
query_tf_interval_seconds : 0.3
valid_hdmap_interval : 1.5
image_sys_ts_diff_threshold : 0.5
sync_interval_seconds : 0.5
default_image_border_size : 100
plugin_param {{
  name: "TLPreprocessor"
}}
gpu_id : 0
proposal_output_channel_name:"/perception/inner/Detection"
"""
