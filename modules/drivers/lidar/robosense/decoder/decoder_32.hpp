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
#define RS32_CHANNELS_PER_BLOCK (32)
#define RS32_BLOCKS_PER_PKT (12)
#define RS32_POINTS_CHANNEL_PER_SECOND (20000)
#define RS32_BLOCKS_CHANNEL_PER_PKT (12)
#define RS32_MSOP_SYNC (0xA050A55A0A05AA55)
#define RS32_BLOCK_ID (0xEEFF)
#define RS32_DIFOP_SYNC (0x555511115A00FFA5)
#define RS32_CHANNEL_TOFFSET (3)
#define RS32_FIRING_TDURATION (50)

typedef struct {
  uint16_t id;
  uint16_t azimuth;
  RS_Channel channels[RS32_CHANNELS_PER_BLOCK];
} __attribute__((packed)) RS32_MsopBlock;

typedef struct {
  RS_MsopHeader header;
  RS32_MsopBlock blocks[RS32_BLOCKS_PER_PKT];
  uint32_t index;
  uint16_t tail;
} __attribute__((packed)) RS32_MsopPkt;

typedef struct {
  uint8_t reserved[240];
  uint8_t coef;
  uint8_t ver;
} __attribute__((packed)) RS32_Intensity;

typedef struct {
  uint64_t sync;
  uint16_t rpm;
  RS_EthNet eth;
  RS_FOV fov;
  uint16_t reserved0;
  uint16_t lock_phase_angle;
  RS_Version version;
  RS32_Intensity intensity;
  RS_SN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RS_Timestamp timestamp;
  RS_Status status;
  uint8_t reserved1[11];
  RS_Diagno diagno;
  uint8_t gprmc[86];
  uint8_t pitch_cali[96];
  uint8_t yaw_cali[96];
  uint8_t reserved2[586];
  uint16_t tail;
} __attribute__((packed)) RS32_DifopPkt;

template <typename vpoint>
class Decoder32 : public DecoderBase<vpoint> {
 public:
  explicit Decoder32(const RS_Param &param);
  int32_t decodeDifopPkt(const uint8_t *pkt);
  int32_t decodeMsopPkt(const uint8_t *pkt,
                        std::shared_ptr<std::vector<vpoint>> vec_ptr,
                        std::shared_ptr<int> height_ptr);
  double getLidarTime(const uint8_t *pkt);
  void loadCalibrationFile(std::string cali_path);
};

template <typename vpoint>
Decoder32<vpoint>::Decoder32(const RS_Param &param)
    : DecoderBase<vpoint>(param) {
  this->Rx_ = 0.03997;
  this->Ry_ = -0.01087;
  this->Rz_ = 0;
  if (this->max_distance_ > 200.0f || this->max_distance_ < 0.4f) {
    this->max_distance_ = 200.0f;
  }
  if (this->min_distance_ > 200.0f ||
      this->min_distance_ > this->max_distance_) {
    this->min_distance_ = 0.4f;
  }
}

template <typename vpoint>
double Decoder32<vpoint>::getLidarTime(const uint8_t *pkt) {
  RS32_MsopPkt *mpkt_ptr =
      const_cast<RS32_MsopPkt *>(reinterpret_cast<const RS32_MsopPkt *>(pkt));

  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
  stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
  stm.tm_mday = mpkt_ptr->header.timestamp.day;
  stm.tm_hour = mpkt_ptr->header.timestamp.hour;
  stm.tm_min = mpkt_ptr->header.timestamp.minute;
  stm.tm_sec = mpkt_ptr->header.timestamp.second;
  return std::mktime(&stm) +
         static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms)) /
             1000.0 +
         static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us)) /
             1000000.0;
}
template <typename vpoint>
int Decoder32<vpoint>::decodeMsopPkt(
    const uint8_t *pkt, std::shared_ptr<std::vector<vpoint>> vec_ptr,
    std::shared_ptr<int> height_ptr) {
  *height_ptr = 32;
  RS32_MsopPkt *mpkt_ptr =
      const_cast<RS32_MsopPkt *>(reinterpret_cast<const RS32_MsopPkt *>(pkt));
  double pkt_time = 0;
  if (this->use_lidar_clock_) {
    std::tm stm;
    memset(&stm, 0, sizeof(stm));
    stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
    stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
    stm.tm_mday = mpkt_ptr->header.timestamp.day;
    stm.tm_hour = mpkt_ptr->header.timestamp.hour;
    stm.tm_min = mpkt_ptr->header.timestamp.minute;
    stm.tm_sec = mpkt_ptr->header.timestamp.second;
    pkt_time =
        std::mktime(&stm) +
        static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms)) /
            1000.0 +
        static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us)) /
            1000000.0;
  }
  if (mpkt_ptr->header.sync != RS32_MSOP_SYNC) {
    return -2;
  }

  int first_azimuth;
  first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  for (int blk_idx = 0; blk_idx < RS32_BLOCKS_PER_PKT; blk_idx++) {
    if (mpkt_ptr->blocks[blk_idx].id != RS32_BLOCK_ID) {
      break;
    }
    int azimuth_blk = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    int azi_prev = 0;
    int azi_cur = 0;
    if (this->echo_mode_ == RS_ECHO_DUAL) {
      if (blk_idx < (RS32_BLOCKS_PER_PKT - 2)) {
        azi_prev = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 2].azimuth);
        azi_cur = azimuth_blk;
      } else {
        azi_prev = azimuth_blk;
        azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 2].azimuth);
      }
    } else {
      if (blk_idx < (RS32_BLOCKS_PER_PKT - 1)) {
        azi_prev = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth);
        azi_cur = azimuth_blk;
      } else {
        azi_prev = azimuth_blk;
        azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth);
      }
    }

    float azimuth_diff =
        static_cast<float>((36000 + azi_prev - azi_cur) % 36000);
    // Ingnore the block if the azimuth change abnormal
    float azimuth_channel;

    for (int channel_idx = 0; channel_idx < RS32_CHANNELS_PER_BLOCK;
         channel_idx++) {
      int azimuth_final;

      azimuth_channel =
          azimuth_blk + (azimuth_diff * RS32_CHANNEL_TOFFSET *
                         (channel_idx % 16) / RS32_FIRING_TDURATION);
      azimuth_final = this->azimuthCalibration(azimuth_channel, channel_idx);

      int idx_map = channel_idx;
      int distance =
          RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[idx_map].distance);
      float intensity = mpkt_ptr->blocks[blk_idx].channels[idx_map].intensity;
      float distancef = distance * RS_RESOLUTION_5mm_DIRSANCE_COEF;
      //
      int angle_horiz_ori;
      int angle_horiz = (azimuth_final + 36000) % 36000;
      int angle_vert;
      angle_horiz_ori = static_cast<int>(azimuth_channel + 36000) % 36000;
      angle_vert =
          ((static_cast<int>(this->vert_angle_list_[channel_idx] * 100) %
            36000) +
           36000) %
          36000;

      // store to pointcloud buffer
      vpoint point;
      if ((distancef <= this->max_distance_ &&
           distancef >= this->min_distance_) &&
          ((this->angle_flag_ && angle_horiz >= this->start_angle_ &&
            angle_horiz <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((angle_horiz >= this->start_angle_ && angle_horiz <= 36000) ||
             (angle_horiz >= 0 && angle_horiz <= this->end_angle_))))) {
        const double vert_cos_value = this->cos_lookup_table_[angle_vert];
        const double horiz_cos_value = this->cos_lookup_table_[angle_horiz];
        const double horiz_ori_cos_value =
            this->cos_lookup_table_[angle_horiz_ori];
        point.set_x(distancef * vert_cos_value * horiz_cos_value +
                    this->Rx_ * horiz_ori_cos_value);

        const double horiz_sin_value = this->sin_lookup_table_[angle_horiz];
        const double horiz_ori_sin_value =
            this->sin_lookup_table_[angle_horiz_ori];
        point.set_y(-distancef * vert_cos_value * horiz_sin_value -
                    this->Rx_ * horiz_ori_sin_value);

        const double vert_sin_value = this->sin_lookup_table_[angle_vert];
        point.set_z(distancef * vert_sin_value + this->Rz_);

        point.set_intensity(intensity);
        if (this->use_lidar_clock_) {
          point.set_timestamp(pkt_time * kSecondToNanoFactor);
        } else {
          point.set_timestamp(cyber::Time().Now().ToSecond() *
                              kSecondToNanoFactor);
        }

        if (std::isnan(intensity)) {
          point.set_intensity(0);
        }
      } else {
        point.set_x(NAN);
        point.set_y(NAN);
        point.set_z(NAN);
        point.set_intensity(0);
      }
      vec_ptr->push_back(point);
    }
  }

  return first_azimuth;
}

template <typename vpoint>
int32_t Decoder32<vpoint>::decodeDifopPkt(const uint8_t *pkt) {
  RS32_DifopPkt *rs32_ptr =
      const_cast<RS32_DifopPkt *>(reinterpret_cast<const RS32_DifopPkt *>(pkt));

  if (rs32_ptr->sync != RS32_DIFOP_SYNC) {
    return -2;
  }

  RS_Version *p_ver = &(rs32_ptr->version);
  if ((p_ver->bottom_rev[0] == 0x08 && p_ver->bottom_rev[1] == 0x02 &&
       p_ver->bottom_rev[2] >= 0x09) ||
      (p_ver->bottom_rev[0] == 0x08 && p_ver->bottom_rev[1] > 0x02) ||
      p_ver->bottom_rev[0] > 0x08) {
    if (rs32_ptr->return_mode == 0x01 || rs32_ptr->return_mode == 0x02) {
      this->echo_mode_ = rs32_ptr->return_mode;
    } else {
      this->echo_mode_ = 0;
    }
  } else {
    this->echo_mode_ = 1;
  }

  int pkt_rate =
      ceil(RS32_POINTS_CHANNEL_PER_SECOND / RS32_BLOCKS_CHANNEL_PER_PKT);

  if (this->echo_mode_ == RS_ECHO_DUAL) {
    pkt_rate = pkt_rate * 2;
  }
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

  if (!(this->cali_data_flag_ & 0x2)) {
    bool angle_flag = true;
    const uint8_t *p_pitch_cali;
    p_pitch_cali = rs32_ptr->pitch_cali;
    if ((p_pitch_cali[0] == 0x00 || p_pitch_cali[0] == 0xFF) &&
        (p_pitch_cali[1] == 0x00 || p_pitch_cali[1] == 0xFF) &&
        (p_pitch_cali[2] == 0x00 || p_pitch_cali[2] == 0xFF)) {
      angle_flag = false;
    }
    if (angle_flag) {
      int lsb, mid, msb, neg = 1;
      const uint8_t *p_yaw_cali = rs32_ptr->yaw_cali;
      for (int i = 0; i < 32; i++) {
        lsb = p_pitch_cali[i * 3];
        if (lsb == 0) {
          neg = 1;
        } else if (lsb == 1) {
          neg = -1;
        }
        mid = p_pitch_cali[i * 3 + 1];
        msb = p_pitch_cali[i * 3 + 2];
        this->vert_angle_list_[i] =
            (mid * 256 + msb) * neg * 0.001f;  // / 180 * M_PI;
        lsb = p_yaw_cali[i * 3];
        if (lsb == 0) {
          neg = 1;
        } else if (lsb == 1) {
          neg = -1;
        }
        mid = p_yaw_cali[i * 3 + 1];
        msb = p_yaw_cali[i * 3 + 2];
        this->hori_angle_list_[i] = (mid * 256 + msb) * neg * 0.001f * 100;
      }
      this->cali_data_flag_ = this->cali_data_flag_ | 0x2;
    }
  }

  return 0;
}

template <typename vpoint>
void Decoder32<vpoint>::loadCalibrationFile(std::string cali_path) {
  int row_index = 0;
  int laser_num = 32;
  std::string line_str;
  this->cali_files_dir_ = cali_path;
  std::string file_dir = this->cali_files_dir_ + "/angle.csv";
  std::ifstream fd(file_dir.c_str(), std::ios::in);
  if (fd.is_open()) {
    row_index = 0;
    while (std::getline(fd, line_str)) {
      std::stringstream ss(line_str);
      std::string str;
      std::vector<std::string> vect_str;
      while (std::getline(ss, str, ',')) {
        vect_str.push_back(str);
      }
      this->vert_angle_list_[row_index] =
          std::stof(vect_str[0]);  // RS_TO_RADS(std::stof(vect_str[0]));
      this->hori_angle_list_[row_index] = std::stof(vect_str[1]) * 100;
      row_index++;
      if (row_index >= laser_num) {
        break;
      }
    }
    fd.close();
  }
}
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
