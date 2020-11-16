/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <cmath>
#include <fstream>
#include <string>

namespace apollo {
namespace drivers {
namespace velodyne {

template <typename T>
void dump_msg(const T& msg, const std::string& file_path) {
  // std::ofstream ofs(file_path.c_str(),
  //                   std::ofstream::out | std::ofstream::binary);
  // uint32_t serial_size = ros::serialization::serializationLength(msg);
  // boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
  // ros::serialization::OStream ostream(obuffer.get(), serial_size);
  // ros::serialization::serialize(ostream, msg);
  // ofs.write((char*)obuffer.get(), serial_size);
  // ofs.close();
}

template <class T>
void load_msg(const std::string& file_path, T* msg) {
  // std::ifstream ifs(file_path.c_str(),
  //                   std::ifstream::in | std::ifstream::binary);
  // ifs.seekg(0, std::ios::end);
  // std::streampos end = ifs.tellg();
  // ifs.seekg(0, std::ios::beg);
  // std::streampos begin = ifs.tellg();
  //
  // uint32_t file_size = end - begin;
  // boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  // ifs.read((char*)ibuffer.get(), file_size);
  // ros::serialization::IStream istream(ibuffer.get(), file_size);
  // ros::serialization::deserialize(istream, *msg);
  // ifs.close();
}

void init_sin_cos_rot_table(float* sin_rot_table, float* cos_rot_table,
                            uint16_t rotation, float rotation_resolution);

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
