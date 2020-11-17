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
#include <utility>
#include <vector>

#include "modules/drivers/lidar/robosense/decoder/decoder_base.hpp"
namespace apollo {
namespace drivers {
namespace robosense {
#pragma pack(push, 1)
typedef struct {
  uint16_t id;
  uint16_t azimuth;
  RSChannel channels[32];
} RS32MsopBlock;

typedef struct {
  RSMsopHeader header;
  RS32MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS32MsopPkt;

typedef struct {
  uint64_t id;
  uint16_t rpm;
  RSEthNet eth;
  RSFOV fov;
  uint16_t reserved0;
  uint16_t phase_lock_angle;
  RSVersion version;
  RSIntensity intensity;
  RSSn sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RSTimestamp timestamp;
  RSStatus status;
  uint8_t reserved1[5];
  RSDiagno diagno;
  uint8_t gprmc[86];
  uint8_t pitch_cali[96];
  uint8_t yaw_cali[96];
  uint8_t reserved2[586];
  uint16_t tail;
} RS32DifopPkt;

#pragma pack(pop)

template <typename T_Point>
class DecoderRS32 : public DecoderBase<T_Point> {
 public:
  explicit DecoderRS32(const RSDecoderParam& param,
                       const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt,
                                std::shared_ptr<std::vector<T_Point>> vec,
                                std::shared_ptr<int> height,
                                std::shared_ptr<int> azimuth);
  double getLidarTime(const uint8_t* pkt);
};

template <typename T_Point>
inline DecoderRS32<T_Point>::DecoderRS32(
    const RSDecoderParam& param,
    const LidarConstantParameter& lidar_const_param)
    : DecoderBase<T_Point>(param, lidar_const_param) {
  this->vert_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->hori_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->beam_ring_table_.resize(this->lidar_const_param_.LASER_NUM);
  if (this->param_.max_distance > 200.0f) {
    this->param_.max_distance = 200.0f;
  }
  if (this->param_.min_distance < 0.4f ||
      this->param_.min_distance > this->param_.max_distance) {
    this->param_.min_distance = 0.4f;
  }
}

template <typename T_Point>
inline double DecoderRS32<T_Point>::getLidarTime(const uint8_t* pkt) {
  return this->template calculateTimeYMD<RS32MsopPkt>(pkt);
}

template <typename T_Point>
inline RSDecoderResult DecoderRS32<T_Point>::decodeMsopPkt(
    const uint8_t* pkt, std::shared_ptr<std::vector<T_Point>> vec,
    std::shared_ptr<int> height, std::shared_ptr<int> azimuth) {
  *height = this->lidar_const_param_.CHANNELS_PER_BLOCK;
  RS32MsopPkt* mpkt_ptr =
      const_cast<RS32MsopPkt*>(reinterpret_cast<const RS32MsopPkt*>(pkt));
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID) {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  *azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  this->current_temperature_ =
      this->computeTemperature(mpkt_ptr->header.temp_raw);
  double block_timestamp = this->get_point_time_func_(pkt);
  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT;
       blk_idx++) {
    if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID) {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    if (this->echo_mode_ == ECHO_DUAL) {
      if (blk_idx % 2 == 0) {
        if (blk_idx == 0) {
          azi_diff = static_cast<float>(
              (RS_ONE_ROUND +
               RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 2].azimuth) - cur_azi) %
              RS_ONE_ROUND);
        } else {
          azi_diff = static_cast<float>(
              (RS_ONE_ROUND + cur_azi -
               RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 2].azimuth)) %
              RS_ONE_ROUND);
          block_timestamp =
              (azi_diff > 100)
                  ? (block_timestamp + this->fov_time_jump_diff_)
                  : (block_timestamp + this->time_duration_between_blocks_);
        }
      }
    } else {
      if (blk_idx == 0) {
        azi_diff = static_cast<float>(
            (RS_ONE_ROUND +
             RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) %
            RS_ONE_ROUND);
      } else {
        azi_diff = static_cast<float>(
            (RS_ONE_ROUND + cur_azi -
             RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth)) %
            RS_ONE_ROUND);
        block_timestamp =
            (azi_diff > 100)
                ? (block_timestamp + this->fov_time_jump_diff_)
                : (block_timestamp + this->time_duration_between_blocks_);
      }
    }
    azi_diff =
        (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff;
    for (int channel_idx = 0;
         channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK;
         channel_idx++) {
      float azi_channel_ori =
          cur_azi +
          azi_diff * this->lidar_const_param_.FIRING_FREQUENCY *
              this->lidar_const_param_.DSR_TOFFSET *
              static_cast<float>(2 * (channel_idx % 16) + (channel_idx / 16));
      int azi_channel_final =
          this->azimuthCalibration(azi_channel_ori, channel_idx);
      float distance =
          RS_SWAP_SHORT(
              mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) *
          RS_RESOLUTION;
      int angle_horiz =
          static_cast<int>(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert =
          ((this->vert_angle_list_[channel_idx]) + RS_ONE_ROUND) % RS_ONE_ROUND;

      T_Point point;
      if ((distance <= this->param_.max_distance &&
           distance >= this->param_.min_distance) &&
          ((this->angle_flag_ && azi_channel_final >= this->start_angle_ &&
            azi_channel_final <= this->end_angle_) ||
           (!this->angle_flag_ && ((azi_channel_final >= this->start_angle_) ||
                                   (azi_channel_final <= this->end_angle_))))) {
        double x =
            distance * this->cos_lookup_table_[angle_vert] *
                this->cos_lookup_table_[azi_channel_final] +
            this->lidar_const_param_.RX * this->cos_lookup_table_[angle_horiz];
        double y =
            -distance * this->cos_lookup_table_[angle_vert] *
                this->sin_lookup_table_[azi_channel_final] -
            this->lidar_const_param_.RX * this->sin_lookup_table_[angle_horiz];
        double z = distance * this->sin_lookup_table_[angle_vert] +
                   this->lidar_const_param_.RZ;
        double intensity =
            mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
        point.set_x(x);
        point.set_y(y);
        point.set_z(z);
        point.set_intensity(intensity);
      } else {
        point.set_x(NAN);
        point.set_y(NAN);
        point.set_z(NAN);
        point.set_intensity(0);
      }
      point.set_timestamp(block_timestamp);
      vec->emplace_back(std::move(point));
    }
  }
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline RSDecoderResult DecoderRS32<T_Point>::decodeDifopPkt(
    const uint8_t* pkt) {
  RS32DifopPkt* dpkt_ptr =
      const_cast<RS32DifopPkt*>(reinterpret_cast<const RS32DifopPkt*>(pkt));
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID) {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  switch (dpkt_ptr->return_mode) {
    case 0x00:
      this->echo_mode_ = RSEchoMode::ECHO_DUAL;
      break;
    case 0x01:
      this->echo_mode_ = RSEchoMode::ECHO_STRONGEST;
      break;
    case 0x02:
      this->echo_mode_ = RSEchoMode::ECHO_LAST;
      break;
    default:
      break;
  }
  this->rpm_ = RS_SWAP_SHORT(dpkt_ptr->rpm);
  this->time_duration_between_blocks_ =
      (60 / static_cast<float>(this->rpm_)) /
      ((this->lidar_const_param_.PKT_RATE * 60 / this->rpm_) *
       this->lidar_const_param_.BLOCKS_PER_PKT);
  int fov_start_angle = RS_SWAP_SHORT(dpkt_ptr->fov.start_angle);
  int fov_end_angle = RS_SWAP_SHORT(dpkt_ptr->fov.end_angle);
  int fov_range = (fov_start_angle < fov_end_angle)
                      ? (fov_end_angle - fov_start_angle)
                      : (RS_ONE_ROUND - fov_start_angle + fov_end_angle);
  int blocks_per_round =
      (this->lidar_const_param_.PKT_RATE / (this->rpm_ / 60)) *
      this->lidar_const_param_.BLOCKS_PER_PKT;
  this->fov_time_jump_diff_ =
      this->time_duration_between_blocks_ *
      (fov_range / (RS_ONE_ROUND / static_cast<float>(blocks_per_round)));
  if (this->echo_mode_ == ECHO_DUAL) {
    this->pkts_per_frame_ =
        ceil(2 * this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  } else {
    this->pkts_per_frame_ =
        ceil(this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  this->azi_diff_between_block_theoretical_ =
      (RS_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_PKT) /
      static_cast<float>(
          this->pkts_per_frame_);  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
  if (!this->difop_flag_) {
    const uint8_t* p_ver_cali = dpkt_ptr->pitch_cali;
    if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) &&
        (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
        (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF)) {
      return RSDecoderResult::DECODE_OK;
    }
    int lsb, mid, msb, neg = 1;
    const uint8_t* p_hori_cali = dpkt_ptr->yaw_cali;
    for (size_t i = 0; i < this->lidar_const_param_.CHANNELS_PER_BLOCK; i++) {
      /* vert angle calibration data */
      lsb = p_ver_cali[i * 3];
      mid = p_ver_cali[i * 3 + 1];
      msb = p_ver_cali[i * 3 + 2];
      neg = lsb == 0 ? 1 : -1;
      this->vert_angle_list_[i] =
          (mid * 256 + msb) * neg * 0.1f;  // / 180 * M_PI;

      /* horizon angle calibration data */
      lsb = p_hori_cali[i * 3];
      mid = p_hori_cali[i * 3 + 1];
      msb = p_hori_cali[i * 3 + 2];
      neg = lsb == 0 ? 1 : -1;
      this->hori_angle_list_[i] = (mid * 256 + msb) * neg * 0.1f;
    }
    this->sortBeamTable();
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
