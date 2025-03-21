#include <cstdio>    // 使用 std::sprintf
#include <sstream>   // 使用 std::ostringstream
#include <iomanip>   // 使用 std::setw 和 std::setfill

// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     <url id="cvd2dhs5rbs5nmss5n20" type="url" status="parsed" title="Apache License, Version 2.0" wc="10467">http://www.apache.org/licenses/LICENSE-2.0</url> 
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/serial/vehicle/ros/ros_parser.h"
#include "cyber/common/log.h"
#include "modules/serial/vehicle/ros/protocol/misc_fb.h"
#include "modules/serial/vehicle/ros/protocol/twist_cmd.h"
#include "modules/serial/vehicle/ros/protocol/twist_fb.h"

namespace apollo {
namespace serial {

bool ROSParser::Encode(const ControlCommand& cmd, uint8_t* data,
                       size_t length) {
    // 输出控制命令的原始值（速度和角速度）
    // ADEBUG << "Received ControlCommand - Speed: " << cmd.speed()
    //       << ", Steering Rate: " << cmd.steering_rate();     
         
    set_x_target_speed(data, cmd.speed());
    set_angular_velocity_z(data, cmd.steering_rate());
    set_checksum(data);

    // // 将编码后的数据帧转换为十六进制字符串用于调试日志输出
    std::ostringstream oss;
    for (size_t i = 0; i < length; ++i) {
        // 使用 std::ostringstream 格式化每个字节
        oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << ' ';
    }
    std::string hex_str = oss.str();
    AERROR << "Encoded data frame: " << hex_str;

    return true;
}

bool ROSParser::DecodeTwistFb(const uint8_t* data, size_t length,
                              Chassis* chassis) {
    chassis->set_speed_mps(x_speed(data, length));
    return true;
}

bool ROSParser::DecodeMiscFb(const uint8_t* data, size_t length,
                             Chassis* chassis) {
    chassis->set_battery_soc_percentage(battery_voltage(data, length));
    return true;
}

}  // namespace serial
}  // namespace apollo