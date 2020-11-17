/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include <memory>
#include <string>

#include "modules/drivers/lidar/robosense/decoder/decoder_128.hpp"
#include "modules/drivers/lidar/robosense/decoder/decoder_16.hpp"
#include "modules/drivers/lidar/robosense/decoder/decoder_32.hpp"
#include "modules/drivers/lidar/robosense/decoder/decoder_80.hpp"
#include "modules/drivers/lidar/robosense/decoder/decoder_bp.hpp"
#include "modules/drivers/lidar/robosense/driver/driver_param.h"

namespace apollo {
namespace drivers {
namespace robosense {
template <typename T_Point>
class DecoderFactory {
 public:
  DecoderFactory() = default;
  ~DecoderFactory() = default;
  static std::shared_ptr<DecoderBase<T_Point>> createDecoder(
      const LidarType& param_lidar_type, const RSDriverParam& param);

 private:
  static std::shared_ptr<DecoderBase<T_Point>> switchLidar(
      const LidarType& lidar_type, const RSDriverParam& param);

  static const LidarConstantParameter getRS16ConstantParam();
  static const LidarConstantParameter getRS32ConstantParam();
  static const LidarConstantParameter getRSBPConstantParam();
  static const LidarConstantParameter getRS80ConstantParam();
  static const LidarConstantParameter getRS128ConstantParam();
};

template <typename T_Point>
inline std::shared_ptr<DecoderBase<T_Point>>
DecoderFactory<T_Point>::createDecoder(const LidarType& param_lidar_type,
                                       const RSDriverParam& param) {
  return switchLidar(param_lidar_type, param);
}

template <typename T_Point>
inline std::shared_ptr<DecoderBase<T_Point>>
DecoderFactory<T_Point>::switchLidar(const LidarType& lidar_type,
                                     const RSDriverParam& param) {
  std::shared_ptr<DecoderBase<T_Point>> ret_ptr;
  switch (lidar_type) {
    case LidarType::RS16:
      ret_ptr = std::make_shared<DecoderRS16<T_Point>>(param.decoder_param,
                                                       getRS16ConstantParam());
      break;
    case LidarType::RS32:
      ret_ptr = std::make_shared<DecoderRS32<T_Point>>(param.decoder_param,
                                                       getRS32ConstantParam());
      break;
    case LidarType::RSBP:
      ret_ptr = std::make_shared<DecoderRSBP<T_Point>>(param.decoder_param,
                                                       getRSBPConstantParam());
      break;
    case LidarType::RS128:
      ret_ptr = std::make_shared<DecoderRS128<T_Point>>(
          param.decoder_param, getRS128ConstantParam());
      break;
    case LidarType::RS80:
      ret_ptr = std::make_shared<DecoderRS80<T_Point>>(param.decoder_param,
                                                       getRS80ConstantParam());
      break;
    default:
      // RS_ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! "
      //          << RS_REND;
      exit(-1);
  }
  ret_ptr->loadCalibrationFile(param.angle_path);
  return ret_ptr;
}

template <typename T_Point>
inline const LidarConstantParameter
DecoderFactory<T_Point>::getRS16ConstantParam() {
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA050A55A0A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 750;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 16;
  ret_param.DSR_TOFFSET = 2.8;
  ret_param.FIRING_FREQUENCY = 0.009;
  ret_param.RX = 0.03825;
  ret_param.RY = -0.01088;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter
DecoderFactory<T_Point>::getRS32ConstantParam() {
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA050A55A0A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 1500;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 32;
  ret_param.DSR_TOFFSET = 1.44;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.03997;
  ret_param.RY = -0.01087;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter
DecoderFactory<T_Point>::getRSBPConstantParam() {
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA050A55A0A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 1500;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 32;
  ret_param.DSR_TOFFSET = 1.28;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.01473;
  ret_param.RY = 0.0085;
  ret_param.RZ = 0.09427;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter
DecoderFactory<T_Point>::getRS80ConstantParam() {
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x5A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xFE;
  ret_param.PKT_RATE = 4500;
  ret_param.BLOCKS_PER_PKT = 4;
  ret_param.CHANNELS_PER_BLOCK = 80;
  ret_param.LASER_NUM = 80;
  ret_param.DSR_TOFFSET = 3.236;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.03615;
  ret_param.RY = -0.017;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter
DecoderFactory<T_Point>::getRS128ConstantParam() {
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x5A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xFE;
  ret_param.PKT_RATE = 6000;
  ret_param.BLOCKS_PER_PKT = 3;
  ret_param.CHANNELS_PER_BLOCK = 128;
  ret_param.LASER_NUM = 128;
  ret_param.DSR_TOFFSET = 3.236;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.03615;
  ret_param.RY = -0.017;
  ret_param.RZ = 0;
  return ret_param;
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
