/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <iostream>

#include "modules/drivers/gnss/parser/asensing_parser/protocol_asensing.h"

Decode_0A::Decode_0A() {
  filename1 = "BDDB0A.csv";
  content1 =
      "GyroX_degps,GyroY_degps,GyroZ_degps,AccX_g,AccY_g,AccZ_g,T_deg[0],"
      "SysTime_ms";

  // createFileAndWrite(filename1, content1);

  registProtocol(m_type, m_length, this);
}

Decode_0A::~Decode_0A() {}

void Decode_0A::subData(const uint8_t* sub_address, int& index) {
  int sub_index = 3;
  uint8_t check_sum = 0;
  int dataLength = getLength(m_type);
  /* check xor */
  for (int i = 0; i < dataLength - 1; ++i) {
    check_sum ^= sub_address[i];
  }

  if (check_sum == sub_address[dataLength - 1]) {
    int16_t middle;
    sub_index = 3;
    // gyro
    float pubMsg_gx = toValue<float>(sub_address, sub_index);
    float m_msg_gx = pubMsg_gx;
    float pubMsg_gy = toValue<float>(sub_address, sub_index);
    float m_msg_gy = pubMsg_gy;
    float pubMsg_gz = toValue<float>(sub_address, sub_index);
    float m_msg_gz = pubMsg_gz;

    // acc
    float pubMsg_ax = toValue<float>(sub_address, sub_index);
    float m_msg_ax = pubMsg_ax;
    float pubMsg_ay = toValue<float>(sub_address, sub_index);
    float m_msg_ay = pubMsg_ay;
    float pubMsg_az = toValue<float>(sub_address, sub_index);
    float m_msg_az = pubMsg_az;

    // temp
    middle = toValue<int16_t>(sub_address, sub_index);
    float pubMsg_temperature = middle * 200.0 / 32768;

    // time
    uint32_t timiddle = toValue<uint32_t>(sub_address, sub_index);
    double time = timiddle * 2.5 * 0.0001;

    index += dataLength;

    // Do not write csv.
    // std::vector<std::string> data_w;
    // data_w.clear();
    // data_w.push_back(std::to_string(pubMsg_gx));
    // data_w.push_back(std::to_string(pubMsg_gy));
    // data_w.push_back(std::to_string(pubMsg_gz));
    // data_w.push_back(std::to_string(pubMsg_ax));
    // data_w.push_back(std::to_string(pubMsg_ay));
    // data_w.push_back(std::to_string(pubMsg_az));
    // data_w.push_back(std::to_string(pubMsg_temperature));
    // data_w.push_back(std::to_string(time));
    // if (AppendCsv(filename1, data_w)) {
    //   // std::cout << "数据成功写入到文件 " << filename << "\n";
    // } else {
    //   std::cerr << "写入文件时出现错误\n";
    // }

  } else {
    index += 3;
  }
}
