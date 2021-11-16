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

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cyber/cyber.h"
namespace apollo {
namespace drivers {
namespace robosense {
static constexpr float kSecondToNanoFactor = 1e9f;
#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)
#define RS_RESOLUTION_5mm_DIRSANCE_COEF (0.005)

enum E_DECODER_RESULT {
  E_DECODE_FAIL = -2,
  E_PARAM_INVALID = -1,
  E_DECODE_OK = 0,
  E_FRAME_SPLIT = 1
};

enum RS_ECHO_MODE { RS_ECHO_DUAL = 0, RS_ECHO_MAX, RS_ECHO_LARS };

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
} __attribute__((packed)) RS_Timestamp;

typedef struct {
  uint64_t sync;
  uint8_t reserved1[12];
  RS_Timestamp timestamp;
  uint8_t lidar_type;
  uint8_t reserved2[7];
  uint16_t temp_raw;
  uint8_t reserved3[2];
} __attribute__((packed)) RS_MsopHeader;

typedef struct {
  uint8_t lidar_ip[4];
  uint8_t host_ip[4];
  uint8_t mac_addr[6];
  uint16_t local_port;
  uint16_t dest_port;
  uint16_t port3;
  uint16_t port4;
} __attribute__((packed)) RS_EthNet;

typedef struct {
  uint16_t start_angle;
  uint16_t end_angle;
} __attribute__((packed)) RS_FOV;
typedef struct {
  uint8_t sign;
  uint8_t value[2];
} __attribute__((packed)) RS_CorAngle;

typedef struct {
  uint16_t distance;
  uint8_t intensity;
} __attribute__((packed)) RS_Channel;

typedef struct {
  uint8_t top_rev[5];
  uint8_t bottom_rev[5];
} __attribute__((packed)) RS_Version;

typedef struct {
  uint8_t num[6];
} __attribute__((packed)) RS_SN;

typedef struct {
  uint8_t device_current[3];
  uint8_t main_current[3];
  uint16_t vol_12v;
  uint16_t vol_12vm;
  uint16_t vol_5v;
  uint16_t vol_3v3;
  uint16_t vol_2v5;
  uint16_t vol_1v2;
} __attribute__((packed)) RS_Status;

typedef struct {
  uint8_t reserved1[10];
  uint8_t checksum;
  uint16_t manc_err1;
  uint16_t manc_err2;
  uint8_t gps_status;
  uint16_t temperature1;
  uint16_t temperature2;
  uint16_t temperature3;
  uint16_t temperature4;
  uint16_t temperature5;
  uint8_t reserved2[5];
  uint16_t cur_rpm;
  uint8_t reserved3[7];
} __attribute__((packed)) RS_Diagno;

typedef struct eRS_Param {
  RS_ECHO_MODE echo = RS_ECHO_MAX;
  float cut_angle = 0.0f;
  float max_distance = 200.0f;
  float min_distance = 0.2f;
  float start_angle = 0.0f;
  float end_angle = 360.0f;
  bool use_lidar_clock = false;
} RS_Param;

//----------------- Decoder ---------------------
template <typename vpoint>
class DecoderBase {
 public:
  explicit DecoderBase(const RS_Param &param);
  virtual ~DecoderBase();
  virtual E_DECODER_RESULT processMsopPkt(
      const uint8_t *pkt, std::shared_ptr<std::vector<vpoint>> vec_ptr,
      std::shared_ptr<int> height_ptr);
  virtual int32_t processDifopPkt(const uint8_t *pkt);
  virtual double getLidarTime(const uint8_t *pkt) = 0;
  virtual void loadCalibrationFile(std::string cali_path) = 0;

 protected:
  virtual int32_t azimuthCalibration(float azimuth, int32_t channel);
  virtual int32_t decodeMsopPkt(const uint8_t *pkt,
                                std::shared_ptr<std::vector<vpoint>> vec_ptr,
                                std::shared_ptr<int> height_ptr) = 0;
  virtual int32_t decodeDifopPkt(const uint8_t *pkt) = 0;

 protected:
  int32_t rpm_;
  float Rx_;
  float Ry_;
  float Rz_;
  float max_distance_;
  float min_distance_;
  bool use_lidar_clock_;
  uint8_t echo_mode_;
  uint8_t channel_num_;
  int start_angle_;
  int end_angle_;
  bool angle_flag_;
  int32_t pkts_per_frame_;
  int32_t pkt_counter_;
  int32_t cut_angle_;
  int32_t last_azimuth_;
  // calibration data
  std::string cali_files_dir_;
  uint32_t cali_data_flag_;
  float vert_angle_list_[128];
  float hori_angle_list_[128];
  float channel_dis_cali_[4][129];
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
};

template <typename vpoint>
DecoderBase<vpoint>::DecoderBase(const RS_Param &param) {
  rpm_ = 600;
  pkts_per_frame_ = 84;
  cali_files_dir_ = ".";
  pkt_counter_ = 0;
  last_azimuth_ = -36001;
  cali_data_flag_ = 0x00;
  angle_flag_ = true;
  start_angle_ = param.start_angle * 100;
  end_angle_ = param.end_angle * 100;
  echo_mode_ = param.echo;
  max_distance_ = param.max_distance;
  min_distance_ = param.min_distance;
  cut_angle_ = param.cut_angle * 100;
  use_lidar_clock_ = param.use_lidar_clock;
  if (cut_angle_ > 36000) {
    cut_angle_ = 0;
  }

  if (this->start_angle_ > 36000 || this->start_angle_ < 0 ||
      this->end_angle_ > 36000 || this->end_angle_ < 0) {
    this->start_angle_ = 0;
    this->end_angle_ = 36000;
  }
  if (this->start_angle_ > this->end_angle_) {
    this->angle_flag_ = false;
  }
  cos_lookup_table_.resize(36000);
  sin_lookup_table_.resize(36000);
  for (unsigned int i = 0; i < 36000; i++) {
    double rad = RS_TO_RADS(i / 100.0f);
    cos_lookup_table_[i] = std::cos(rad);
    sin_lookup_table_[i] = std::sin(rad);
  }
}

template <typename vpoint>
DecoderBase<vpoint>::~DecoderBase() {
  this->cos_lookup_table_.clear();
  this->sin_lookup_table_.clear();
}

template <typename vpoint>
int32_t DecoderBase<vpoint>::processDifopPkt(const uint8_t *pkt) {
  if (pkt == NULL) {
    return -1;
  }
  return decodeDifopPkt(pkt);
}

template <typename vpoint>
E_DECODER_RESULT DecoderBase<vpoint>::processMsopPkt(
    const uint8_t *pkt, std::shared_ptr<std::vector<vpoint>> vec_ptr,
    std::shared_ptr<int> height_ptr) {
  if (pkt == NULL) {
    return E_PARAM_INVALID;
  }

  int azimuth = decodeMsopPkt(pkt, vec_ptr, height_ptr);
  if (azimuth < 0) {
    return E_DECODE_FAIL;
  }
  this->pkt_counter_++;
  if (this->cut_angle_ >= 0) {
    if (azimuth < this->last_azimuth_) {
      this->last_azimuth_ -= 36000;
    }
    if (this->last_azimuth_ != -36001 &&
        this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_) {
      this->last_azimuth_ = azimuth;
      this->pkt_counter_ = 0;
      return E_FRAME_SPLIT;
    }
    this->last_azimuth_ = azimuth;
  } else {
    if (this->pkt_counter_ >= this->pkts_per_frame_) {
      this->pkt_counter_ = 0;
      return E_FRAME_SPLIT;
    }
  }

  return E_DECODE_OK;
}

template <typename vpoint>
int DecoderBase<vpoint>::azimuthCalibration(float azimuth, int channel) {
  int azi_ret;

  if (azimuth > 0.0 && azimuth < 3000.0) {
    azimuth = azimuth + this->hori_angle_list_[channel] + 36000.0f;
  } else {
    azimuth = azimuth + this->hori_angle_list_[channel];
  }
  azi_ret = static_cast<int>(azimuth);
  azi_ret %= 36000;

  return azi_ret;
}
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
