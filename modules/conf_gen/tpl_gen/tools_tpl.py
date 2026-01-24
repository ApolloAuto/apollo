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
Tools Template
"""

IMAGE_CREATOR_CONF_TPL = """
reader_conf {{
   point_cloud_channel: "{point_cloud_channel}"
   localization_channel: "/apollo/localization/pose"
}}

traffic_light_detection_conf {{
    camera_frame_id: "{traffic_light_camera_frame_id}"
    camera_channel: "{traffic_light_camera_channel}"
    camera_intrinsic_file_path: "{traffic_light_camera_intrinsics}"
    car2stopline_max_distance: {car2stopline_max_distance}
    car2stopline_max_deg: {car2stopline_max_deg}
}}

point_cloud_processing_conf {{
    intensity_converter_conf {{
        enable_sigmoid_converter: true
        intensity_sigmoid_k: {intensity_sigmoid_k}
        intensity_sigmoid_x0: {intensity_sigmoid_x0}
    }}
    filter_conf {{
        enable_height_filter: true
        upper_height_limit_relative_to_pose: {upper_height_limit_relative_to_pose}
        lower_height_limit_relative_to_pose: {lower_height_limit_relative_to_pose}

        enable_distance_filter: true
        upper_distance_limit: {upper_distance_limit}
        lower_distance_limit: {lower_distance_limit}
    }}
    vertical_segmentation {{
        enable_height_relative_converter: {enable_height_relative_converter}
        height_var_k: {height_var_k}
        height_count_threshold: {height_count_threshold}
        height_var_threshold: {height_var_threshold}
    }}
}}

matrix_generator_conf {{
    matrix_resolution: 0.03125
    matrix_id: 4
}}

input_output_conf {{
    input_dir: "/apollo_workspace/data/record"
    bin_output_dir: "/apollo/data/base_map/sample/map_bin"
    images_output_dir: "/apollo/data/base_map/sample/map_images"
    use_LRU_cache: true
    LRU_cache_size: 20
}}

slam_mode_selection_conf {{
    enable_slam_mode: false
    slam_pose_path: "/apollo/data/slam_tf/sample/slam_pose_result.bin"
}}

coordinate_transformer_conf {{
    worker_num: 12
}}

sample_distance: {sample_distance}

debug_conf {{
    enable_pcd_file_output: false
    debug_pcd_file_path: "/apollo_workspace/data/debug.pcd"
}}
"""

LIDAR_CALIBRATION_CONF_TPL = """
        {{ 
            name: "{frame_id}" # lidar名字，前端展示用
            lidar_frame_id: "{frame_id}"
            lidar_channel: "{point_cloud_channel}" # lidar驱动输出的channel名字
            extrinsics {{ # 初始外参的旋转矩阵和平移矩阵
                qw: {rotation_w}
                qx: {rotation_x}
                qy: {rotation_y}
                qz: {rotation_z}
                tx: {translation_x}
                ty: {translation_y}
                tz: {translation_z}
            }}
            output_filename: "{output_filename}" # 标定结果文件路径(文件要存在)
        }}"""

CAMERA_CALIBRATION_CONF_TPL = """
        {{
            name: "{frame_id}" # camera名字，前端展示用
            camera_channel: "{channel_name}" # camera动输出的channel名字
            camera_frame_id: "{frame_id}"
            lidar_channel: "{lidar_channel}" # lidar驱动输出的channel名字
            lidar_frame_id: "{lidar_frame_id}"
            lidar_rotation: {lidar_rotation} # lidar点云旋转角度，默认是0度，逆时针为正
            intrinsics_filename: "{intrinsics_filename}" # 内参文件路径
            output_filename: "{output_filename}" # 标定结果文件路径(文件要存在)
            lidar_to_imu_extrinsic_file_path: "{lidar_extrinsics_filename}" #lidar-imu外参路径
        }}"""

CALIBRATION_CONF_TPL = """
lidar_calib {{ # lidar标定配置
    collect_strategy {{ # 标定数据采集策略
        angle: {angle} # 定位信息和前一帧角度差超过这个阈值作为一个有效帧记录，取值：度数/180°*π
        distance: {distance} # 定位信息和前一帧距离差超过这个阈值作为一个有效帧记录， angle和distance是或的关系，满足一个就记录
        total: {total} # 采集的帧数
    }}
    use_odometry: true # pose信息源，true（默认）：/apollo/sensor/gnss/odometry，false：/apollo/localization/pose
    calibration_data_path: "/apollo/data/calibration/lidar_calibration"  # 每次标定创建{{calibration_data_path}}/{{task_id}}目录，里面包括采集数据(collection_data)和标定结果(result)目录
    imu_frame_id: "imu"
    lidar_list [ # lidar列表
        {lidar_list}
    ]
    calib_height: {calib_height} # 是否标高度，true是，false否
}}

camera_calib {{ # camera标定配置
    calibration_data_path: "/apollo/data/calibration/camera_calibration"  ## 每次标定创建{{calibration_data_path}}/{{task_id}}目录，里面包括采集数据(collection_data)和标定结果(result)目录
    camera_list [ # camera列表
        {camera_list}
    ]
}}
"""
