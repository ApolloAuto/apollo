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
  uint8_t id;
  uint8_t ret_id;
  uint16_t azimuth;
  RSChannel channels[128];
} RS128MsopBlock;

typedef struct {
  unsigned int id;
  uint8_t reserved1[3];
  uint8_t wave_mode;
  uint8_t temp_low;
  uint8_t temp_high;
  RSTimestampUTC timestamp_utc;
  uint8_t reserved2[10];
  uint8_t lidar_type;
  uint8_t reserved3[49];
} RS128MsopHeader;

typedef struct {
  RS128MsopHeader header;
  RS128MsopBlock blocks[3];
  unsigned int index;
} RS128MsopPkt;

typedef struct {
  uint8_t sync_mode;
  uint8_t sync_sts;
  RSTimestamp timestamp;
} RS128TimeInfo;

typedef struct {
  uint8_t lidar_ip[4];
  uint8_t dest_ip[4];
  uint8_t mac_addr[6];
  uint16_t msop_port;
  uint16_t reserve_1;
  uint16_t difop_port;
  uint16_t reserve_2;
} RS128EthNet;

typedef struct {
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
} RS128Version;

typedef struct {
  uint64_t id;
  uint16_t rpm;
  RS128EthNet eth;
  RSFOV fov;
  uint16_t reserved_0;
  uint16_t phase_lock_angle;
  RS128Version version;
  uint8_t reserved_1[229];
  RSSn sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  RS128TimeInfo time_info;
  RSStatus status;
  uint8_t reserved_2[5];
  RSDiagno diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle ver_angle_cali[128];
  RSCalibrationAngle hori_angle_cali[128];
  uint8_t reserved_3[10];
  uint16_t tail;
} RS128DifopPkt;

#pragma pack(pop)

template <typename T_Point>
class DecoderRS128 : public DecoderBase<T_Point> {
 public:
  explicit DecoderRS128(const RSDecoderParam& param,
                        const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt,
                                std::shared_ptr<std::vector<T_Point>> vec,
                                std::shared_ptr<int> height,
                                std::shared_ptr<int> azimuth);
  double getLidarTime(const uint8_t* pkt);
};

template <typename T_Point>
inline DecoderRS128<T_Point>::DecoderRS128(
    const RSDecoderParam& param,
    const LidarConstantParameter& lidar_const_param)
    : DecoderBase<T_Point>(param, lidar_const_param) {
  this->vert_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->hori_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->beam_ring_table_.resize(this->lidar_const_param_.LASER_NUM);
  if (this->param_.max_distance > 250.0f) {
    this->param_.max_distance = 250.0f;
  }
  if (this->param_.min_distance < 1.0f ||
      this->param_.min_distance > this->param_.max_distance) {
    this->param_.min_distance = 1.0f;
  }
}

template <typename T_Point>
inline double DecoderRS128<T_Point>::getLidarTime(const uint8_t* pkt) {
  return this->template calculateTimeUTC<RS128MsopPkt>(pkt);
}

template <typename T_Point>
inline RSDecoderResult DecoderRS128<T_Point>::decodeMsopPkt(
    const uint8_t* pkt, std::shared_ptr<std::vector<T_Point>> vec,
    std::shared_ptr<int> height, std::shared_ptr<int> azimuth) {
  *height = this->lidar_const_param_.LASER_NUM;
  RS128MsopPkt* mpkt_ptr =
      const_cast<RS128MsopPkt*>(reinterpret_cast<const RS128MsopPkt*>(pkt));
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID) {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  *azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  this->current_temperature_ = this->computeTemperature(
      mpkt_ptr->header.temp_low, mpkt_ptr->header.temp_high);
  double block_timestamp = this->get_point_time_func_(pkt);
  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT;
       blk_idx++) {
    if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID) {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    if (this->echo_mode_ == ECHO_DUAL) {
      azi_diff = static_cast<float>(
          (RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[2].azimuth) -
           RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth)) %
          RS_ONE_ROUND);
      if (RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth) ==
          RS_SWAP_SHORT(mpkt_ptr->blocks[1].azimuth)) {
        if (blk_idx == 2) {
          block_timestamp =
              (azi_diff > 100)
                  ? (block_timestamp + this->fov_time_jump_diff_)
                  : (block_timestamp + this->time_duration_between_blocks_);
        }
      } else {
        if (blk_idx == 1) {
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
    for (size_t channel_idx = 0;
         channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK;
         channel_idx++) {
      int dsr_temp = (channel_idx / 4) % 16;
      float azi_channel_ori = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth) +
                              (azi_diff * static_cast<float>(dsr_temp) *
                               this->lidar_const_param_.DSR_TOFFSET *
                               this->lidar_const_param_.FIRING_FREQUENCY);
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
inline RSDecoderResult DecoderRS128<T_Point>::decodeDifopPkt(
    const uint8_t* pkt) {
  RS128DifopPkt* dpkt_ptr =
      const_cast<RS128DifopPkt*>(reinterpret_cast<const RS128DifopPkt*>(pkt));
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID) {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  this->echo_mode_ = (RSEchoMode)dpkt_ptr->return_mode;
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
  if (this->echo_mode_ == RSEchoMode::ECHO_DUAL) {
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
    const uint8_t* p_ver_cali = const_cast<uint8_t*>(
        reinterpret_cast<const uint8_t*>(dpkt_ptr->ver_angle_cali));
    if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) &&
        (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
        (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) &&
        (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF)) {
      return RSDecoderResult::DECODE_OK;
    }

    int lsb, mid, msb, neg = 1;
    for (size_t i = 0; i < this->lidar_const_param_.CHANNELS_PER_BLOCK; i++) {
      // calculation of vertical angle
      lsb = dpkt_ptr->ver_angle_cali[i].sign;
      mid = dpkt_ptr->ver_angle_cali[i].value[0];
      msb = dpkt_ptr->ver_angle_cali[i].value[1];
      neg = lsb == 0 ? 1 : -1;
      this->vert_angle_list_[i] = (mid * 256 + msb) * neg;  // * 0.01f;

      // horizontal offset angle
      lsb = dpkt_ptr->hori_angle_cali[i].sign;
      mid = dpkt_ptr->hori_angle_cali[i].value[0];
      msb = dpkt_ptr->hori_angle_cali[i].value[1];
      neg = lsb == 0 ? 1 : -1;
      this->hori_angle_list_[i] = (mid * 256 + msb) * neg;  // * 0.01f;
    }
    this->sortBeamTable();
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
