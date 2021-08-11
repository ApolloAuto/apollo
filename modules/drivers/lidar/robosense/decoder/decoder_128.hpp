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
#define RS128_MSOP_SYNC (0x5A05AA55)  // big endian
#define RS128_BLOCK_ID (0xFE)
#define RS128_DIFOP_SYNC (0x555511115A00FFA5)  // big endian
#define RS128_CHANNELS_PER_BLOCK (128)
#define RS128_BLOCKS_PER_PKT (3)
#define RS128_DSR_TOFFSET (3.0)
#define RS128_BLOCK_TDURATION (55.0)

typedef struct {
  uint8_t id;
  uint8_t ret_id;
  uint16_t azimuth;
  RS_Channel channels[RS128_CHANNELS_PER_BLOCK];
} __attribute__((packed)) RS128_MsopBlock;

typedef struct {
  uint32_t sync;
  uint8_t reserved1[3];
  uint8_t wave_mode;
  uint8_t temp_low;
  uint8_t temp_high;
  RS_Timestamp timestamp;
  uint8_t reserved2[60];
} __attribute__((packed)) RS128_MsopHeader;

typedef struct {
  RS128_MsopHeader header;
  RS128_MsopBlock blocks[RS128_BLOCKS_PER_PKT];
  uint32_t index;
} __attribute__((packed)) RS128_MsopPkt;

typedef struct {
  uint8_t reserved[229];
} __attribute__((packed)) RS128_Reserved;

typedef struct {
  uint8_t sync_mode;
  uint8_t sync_sts;
  RS_Timestamp timestamp;
} __attribute__((packed)) RS128_TimeInfo;

typedef struct {
  uint8_t lidar_ip[4];
  uint8_t dest_ip[4];
  uint8_t mac_addr[6];
  uint16_t msop_port;
  uint16_t reserve_1;
  uint16_t difop_port;
  uint16_t reserve_2;
} __attribute__((packed)) RS128_EthNet;

typedef struct {
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
} __attribute__((packed)) RS128_Version;

typedef struct {
  uint64_t sync;
  uint16_t rpm;
  RS128_EthNet eth;
  RS_FOV fov;
  uint16_t reserved_0;
  uint16_t lock_phase_angle;
  RS128_Version version;
  uint8_t reserved_1[229];
  RS_SN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  RS128_TimeInfo time_info;
  RS_Status status;
  uint8_t reserved_2[11];
  RS_Diagno diagno;
  uint8_t gprmc[86];
  RS_CorAngle ver_angle_cali[128];
  RS_CorAngle hori_angle_cali[128];
  uint8_t reserved_3[10];
  uint16_t tail;
} __attribute__((packed)) RS128_DifopPkt;

template <typename vpoint>
class Decoder128 : public DecoderBase<vpoint> {
 public:
  explicit Decoder128(const RS_Param &param);
  int32_t decodeDifopPkt(const uint8_t *pkt);
  int32_t decodeMsopPkt(const uint8_t *pkt,
                        std::shared_ptr<std::vector<vpoint>> vec_ptr,
                        std::shared_ptr<int> height_ptr);
  double getLidarTime(const uint8_t *pkt);
  void loadCalibrationFile(std::string cali_path);

 private:
  int32_t azimuthCalibration(float azimuth, int32_t channel);
};

template <typename vpoint>
Decoder128<vpoint>::Decoder128(const RS_Param &param)
    : DecoderBase<vpoint>(param) {
  this->Rx_ = 0.03615;
  this->Ry_ = -0.017;
  this->Rz_ = 0;
  this->channel_num_ = 128;
  if (param.max_distance > 250.0f || param.max_distance < 1.0f) {
    this->max_distance_ = 250.0f;
  } else {
    this->max_distance_ = param.max_distance;
  }

  if (param.min_distance > 250.0f || param.min_distance > param.max_distance) {
    this->min_distance_ = 1.0f;
  } else {
    this->min_distance_ = param.min_distance;
  }

  int pkt_rate = 6000;
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);
}

template <typename vpoint>
double Decoder128<vpoint>::getLidarTime(const uint8_t *pkt) {
  RS128_MsopPkt *mpkt_ptr =
      const_cast<RS128_MsopPkt *>(reinterpret_cast<const RS128_MsopPkt *>(pkt));
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
int Decoder128<vpoint>::decodeMsopPkt(
    const uint8_t *pkt, std::shared_ptr<std::vector<vpoint>> vec_ptr,
    std::shared_ptr<int> height_ptr) {
  *height_ptr = 128;
  RS128_MsopPkt *mpkt_ptr =
      const_cast<RS128_MsopPkt *>(reinterpret_cast<const RS128_MsopPkt *>(pkt));
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
  if (mpkt_ptr->header.sync != RS128_MSOP_SYNC) {
    return -2;
  }

  float azimuth_corrected_float;
  int azimuth_corrected;
  int first_azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);

  for (int blk_idx = 0; blk_idx < RS128_BLOCKS_PER_PKT; blk_idx++) {
    if (mpkt_ptr->blocks[blk_idx].id != RS128_BLOCK_ID) {
      break;
    }

    int azimuth_blk = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    int azi_prev = 0;
    int azi_cur = 0;
    if (this->echo_mode_ == RS_ECHO_DUAL) {
      azi_prev = RS_SWAP_SHORT(mpkt_ptr->blocks[2].azimuth);
      azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);

    } else {
      if (blk_idx < (RS128_BLOCKS_PER_PKT - 1)) {
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
    if (azimuth_diff <= 0.0 || azimuth_diff > 40.0) {
      continue;
    }

    for (int channel_idx = 0; channel_idx < RS128_CHANNELS_PER_BLOCK;
         channel_idx++) {
      int dsr_temp;
      if (channel_idx >= 16) {
        dsr_temp = channel_idx % 16;
      } else {
        dsr_temp = channel_idx;
      }

      azimuth_corrected_float =
          azimuth_blk + (azimuth_diff * (dsr_temp * RS128_DSR_TOFFSET) /
                         RS128_BLOCK_TDURATION);
      azimuth_corrected =
          this->azimuthCalibration(azimuth_corrected_float, channel_idx);

      int distance = RS_SWAP_SHORT(
          mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance);

      float intensity =
          mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
      float distancef = distance * RS_RESOLUTION_5mm_DIRSANCE_COEF;
      int angle_horiz_ori =
          static_cast<int>(azimuth_corrected_float + 36000) % 36000;
      int angle_horiz = (azimuth_corrected + 36000) % 36000;
      int angle_vert =
          ((static_cast<int>(this->vert_angle_list_[channel_idx] * 100) %
            36000) +
           36000) %
          36000;

      vpoint point;
      if ((distancef > this->max_distance_ ||
           distancef < this->min_distance_) ||
          (this->angle_flag_ && (angle_horiz < this->start_angle_ ||
                                 angle_horiz > this->end_angle_)) ||
          (!this->angle_flag_ && (angle_horiz > this->start_angle_ &&
                                  angle_horiz < this->end_angle_))) {
        point.set_x(NAN);
        point.set_y(NAN);
        point.set_z(NAN);
        point.set_intensity(0);
      } else {
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
      }
      vec_ptr->push_back(std::move(point));
    }
  }

  return first_azimuth;
}

template <typename vpoint>
int Decoder128<vpoint>::decodeDifopPkt(const uint8_t *pkt) {
  RS128_DifopPkt *rs128_ptr = const_cast<RS128_DifopPkt *>(
      reinterpret_cast<const RS128_DifopPkt *>(pkt));

  if (rs128_ptr->sync != RS128_DIFOP_SYNC) {
    return -2;
  }

  int pkt_rate = 6760;
  this->rpm_ = rs128_ptr->rpm;
  this->echo_mode_ = rs128_ptr->return_mode;

  if (this->echo_mode_ == RS_ECHO_DUAL) {
    pkt_rate = pkt_rate * 2;
  }
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

  if (!(this->cali_data_flag_ & 0x2)) {
    bool angle_flag = true;
    const uint8_t *p_ver_cali;
    p_ver_cali = reinterpret_cast<uint8_t *>(rs128_ptr->ver_angle_cali);
    if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) &&
        (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
        (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) &&
        (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF)) {
      angle_flag = false;
    }

    if (angle_flag) {
      int lsb, mid, msb, neg = 1;
      for (int i = 0; i < 128; i++) {
        // calculation of vertical angle
        lsb = rs128_ptr->ver_angle_cali[i].sign;
        mid = rs128_ptr->ver_angle_cali[i].value[0];
        msb = rs128_ptr->ver_angle_cali[i].value[1];
        if (lsb == 0) {
          neg = 1;
        } else {
          neg = -1;
        }
        this->vert_angle_list_[i] = (mid * 256 + msb) * neg * 0.01f;

        // horizontal offset angle
        lsb = rs128_ptr->hori_angle_cali[i].sign;
        mid = rs128_ptr->hori_angle_cali[i].value[0];
        msb = rs128_ptr->hori_angle_cali[i].value[1];
        if (lsb == 0) {
          neg = 1;
        } else {
          neg = -1;
        }
        this->hori_angle_list_[i] = (mid * 256 + msb) * neg * 0.01f;
      }
      this->cali_data_flag_ = this->cali_data_flag_ | 0x2;
    }
  }
  return 0;
}

template <typename vpoint>
void Decoder128<vpoint>::loadCalibrationFile(std::string cali_path) {
  int row_index = 0;
  int laser_num = 128;
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
          std::stof(vect_str[0]); /*  / 180 * M_PI */
      this->hori_angle_list_[row_index] = std::stof(vect_str[1]) * 100;
      row_index++;
      if (row_index >= laser_num) {
        break;
      }
    }
    fd.close();
  }
}
template <typename vpoint>
int Decoder128<vpoint>::azimuthCalibration(float azimuth, int channel) {
  int azi_ret;

  if (azimuth > 0.0 && azimuth < 3000.0) {
    azimuth = azimuth + this->hori_angle_list_[channel] * 100 + 36000.0f;
  } else {
    azimuth = azimuth + this->hori_angle_list_[channel] * 100;
  }
  azi_ret = static_cast<int>(azimuth);
  azi_ret %= 36000;

  return azi_ret;
}
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
