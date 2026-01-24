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
Transform Template
"""

STATIC_TRANSFORM_CONF_TPL = """extrinsic_file {{
    frame_id: "{parent_frame_id}"
    child_frame_id: "{child_frame_id}"
    file_path: "{file_path}"
    enable: true
}}
"""

EXTRINSIC_PARAMS_TPL = """
header:
  frame_id: {parent_frame_id}
transform:
  rotation:
    x: {rotation_x}
    y: {rotation_y}
    z: {rotation_z}
    w: {rotation_w}
  translation:
    x: {translation_x}
    y: {translation_y}
    z: {translation_z}
child_frame_id: {child_frame_id}
"""
