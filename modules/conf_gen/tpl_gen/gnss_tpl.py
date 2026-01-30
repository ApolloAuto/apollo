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
Gnss Template
"""

GNSS_DATA_TPL = """
data {{
    # format optional: NOVATEL_BINARY, HUACE_TEXT, ASENSING_BINARY, BROADGNSS_TEXT
    format: {format}

    # protocol optional: serial, tcp, udp, ntrip, can_card_parameter
    {protocol}
}}
"""

GNSS_RTK_FROM_TPL = """
rtk_from {{
    format: {format}
    ntrip {{
        address: "{address}"
        port: {port}
        mount_point: "{mount_point}"
        user: "{user}"
        password: "{password}"
        timeout_s: {timeout_s}
    }}
    push_location: {push_location}
}}
"""

GNSS_RTK_TO_TPL = """
rtk_to {{
    format: {format}
    {protocol}
    is_data_stream: {is_data_stream}
}}
"""

GNSS_COMMAND_TPL = """
command {{
    format: {format}
    {protocol}
}}
"""

GNSS_SERIAL_PROTOCOL_TPL = """
    serial {{
        device: "{device}"
        baud_rate: {baud_rate}
    }}
"""

GNSS_TCP_PROTOCOL_TPL = """
    tcp {{
        address: "{address}"
        port: {port}
    }}
"""

GNSS_UDP_PROTOCOL_TPL = """
    udp {{
        address: "{address}"
        port: {port}
    }}
"""

GNSS_CAN_PROTOCOL_TPL = """
    can_card_parameter {{
       brand: {brand}
       type: {type}
       channel_id: {channel_id}
       interface: {interface}
    }}
"""

GNSS_CONF_TPL = """
{data_content}{rtk_from_content}{rtk_to_content}{command_content}

rtk_solution_type: {rtk_solution_type}
imu_type: {imu_type}
proj4_text: "+proj=utm +zone={zone_id} +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs"

tf {{
    frame_id: "world"
    child_frame_id: "imu"
}}

# If given, the driver will send velocity info into novatel one time per second
# wheel_parameters: "SETWHEELPARAMETERS 100 1 1\\r\\n"

gpsbin_folder: "/apollo/data/gpsbin"

use_gnss_time: {use_gnss_time}
"""
