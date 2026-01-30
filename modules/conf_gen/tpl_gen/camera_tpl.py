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
Camera Template
"""

CAMERA_CONF_TPL = """
camera_dev: "{camera_dev}"
frame_id: "{frame_id}"
pixel_format: "{pixel_format}"
io_method: {io_method}
width: {width}
height: {height}
frame_rate: {frame_rate}
monochrome: {monochrome}
brightness: {brightness}
contrast: {contrast}
saturation: {saturation}
sharpness: {sharpness}
gain: {gain}
auto_focus: {auto_focus}
focus: {focus}
auto_exposure: {auto_exposure}
exposure: {exposure}
auto_white_balance: {auto_white_balance}
white_balance: {white_balance}
bytes_per_pixel: {bytes_per_pixel}
trigger_internal: {trigger_internal}
trigger_fps: {trigger_fps}
raw_channel_name: "{raw_channel_name}"
channel_name: "{channel_name}"
device_wait_ms: {device_wait_ms}
spin_rate: {spin_rate}
output_type: {output_type}

compress_conf {{
    output_channel: "{compress_channel}"
    image_pool_size: 100
}}

hardware_trigger: false

time_compensator_conf {{
    enable_compensator: true
    compensator_fnode: "/sys/devices/system/clocksource/clocksource0/offset_ns"
}}

arm_gpu_acceleration: true
"""

CAMERA_COMPONENT_TPL = """
    components {{
      class_name : "CameraComponent"
      config {{
        name : "{frame_id}"
        config_file_path : "{config_file_path}"
      }}
    }}
"""

CAMERA_COMPRESS_COMPONENT_TPL = """
    components {{
      class_name : "CompressComponent"
      config {{
        name : "{frame_id}_compress"
        config_file_path : "{config_file_path}"
        readers {{
          channel: "{raw_channel_name}"
          pending_queue_size: 10
        }}
      }}
    }}
"""

CAMERA_DAG_TPL = """
module_config {{
    module_library : "modules/drivers/camera/libcamera_component.so"
    {camera_components}
}}

module_config {{
    module_library : "modules/drivers/camera/libcamera_compress_component.so"
    {camera_compress_components}
}}
"""
