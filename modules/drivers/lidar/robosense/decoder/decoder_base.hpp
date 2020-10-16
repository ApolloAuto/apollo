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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/robosense/driver/driver_param.h"
namespace apollo {
namespace drivers {
namespace robosense {
static constexpr float kSecondToNanoFactor = 1e9f;
#define DEFINE_MEMBER_CHECKER(member)                                        \
  template <typename T, typename V = bool>                                   \
  struct has_##member : std::false_type {};                                  \
  template <typename T>                                                      \
  struct has_##member<                                                       \
      T, typename std::enable_if<                                            \
             !std::is_same<decltype(std::declval<T>().member), void>::value, \
             bool>::type> : std::true_type {};
#define HAS_MEMBER(C, member) has_##member<C>::value
DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)
#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_SWAP_LONG(x)                                               \
  ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | \
  (((x)&0xFF000000) >> 24))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)
const double RS_RESOLUTION = 0.005;
const int RS_ONE_ROUND = 36000;
/* Echo mode definition */
enum RSEchoMode { ECHO_LAST = 1, ECHO_STRONGEST, ECHO_DUAL };

/* Decode result definition */
enum RSDecoderResult {
  DECODE_OK = 0,
  FRAME_SPLIT = 1,
  WRONG_PKT_HEADER = -1,
  PKT_NULL = -2
};

#pragma pack(push, 1)
typedef struct {
  uint64_t MSOP_ID;
  uint64_t DIFOP_ID;
  uint64_t BLOCK_ID;
  uint32_t PKT_RATE;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;
  uint16_t LASER_NUM;
  float DSR_TOFFSET;
  float FIRING_FREQUENCY;
  float RX;
  float RY;
  float RZ;
} LidarConstantParameter;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
} RSTimestamp;

typedef struct {
  uint8_t sec[6];
  unsigned int ns;
} RSTimestampUTC;

typedef struct {
  uint64_t id;
  uint8_t reserved1[12];
  RSTimestamp timestamp;
  uint8_t lidar_type;
  uint8_t reserved2[7];
  uint16_t temp_raw;
  uint8_t reserved3[2];
} RSMsopHeader;

typedef struct {
  uint8_t lidar_ip[4];
  uint8_t host_ip[4];
  uint8_t mac_addr[6];
  uint16_t local_port;
  uint16_t dest_port;
  uint16_t port3;
  uint16_t port4;
} RSEthNet;

typedef struct {
  uint16_t start_angle;
  uint16_t end_angle;
} RSFOV;

typedef struct {
  uint8_t sign;
  uint8_t value[2];
} RSCalibrationAngle;

typedef struct {
  uint16_t distance;
  uint8_t intensity;
} RSChannel;

typedef struct {
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
} RSVersion;

typedef struct {
  uint8_t num[6];
} RSSn;

typedef struct {
  uint8_t reserved[240];
  uint8_t coef;
  uint8_t ver;
} RSIntensity;

typedef struct {
  uint8_t device_current[3];
  uint8_t main_current[3];
  uint16_t vol_12v;
  uint16_t vol_sim_1v8;
  uint16_t vol_dig_3v3;
  uint16_t vol_sim_3v3;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_ejc_5v;
  uint16_t vol_recv_5v;
  uint16_t vol_apd;
} RSStatus;

typedef struct {
  uint8_t reserved1[9];
  uint16_t checksum;
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
} RSDiagno;
#pragma pack(pop)
typedef std::pair<std::string, double> CameraTrigger;

//----------------- Decoder ---------------------
template <typename T_Point>
class DecoderBase {
 public:
  explicit DecoderBase(const RSDecoderParam& param,
                       const LidarConstantParameter& lidar_const_param);
  DecoderBase(const DecoderBase&) = delete;
  DecoderBase& operator=(const DecoderBase&) = delete;
  virtual ~DecoderBase() = default;
  virtual RSDecoderResult processMsopPkt(
      const uint8_t* pkt, std::shared_ptr<std::vector<T_Point>> point_cloud_vec,
      std::shared_ptr<int> height);
  virtual RSDecoderResult processDifopPkt(const uint8_t* pkt);
  virtual void loadCalibrationFile(const std::string& angle_path);
  virtual void regRecvCallback(const std::function<void(const CameraTrigger&)>&
                                   callback);  ///< Camera trigger
  virtual double getLidarTemperature();
  virtual double getLidarTime(const uint8_t* pkt) = 0;

 protected:
  virtual float computeTemperature(const uint16_t& temp_raw);
  virtual float computeTemperature(const uint8_t& temp_low,
                                   const uint8_t& temp_high);
  virtual int azimuthCalibration(const float& azimuth, const int& channel);
  virtual void checkTriggerAngle(const int& angle, const double& timestamp);
  virtual RSDecoderResult decodeMsopPkt(
      const uint8_t* pkt, std::shared_ptr<std::vector<T_Point>> vec,
      std::shared_ptr<int> height, std::shared_ptr<int> azimuth) = 0;
  virtual RSDecoderResult decodeDifopPkt(const uint8_t* pkt) = 0;
  template <typename T_Msop>
  double calculateTimeUTC(const uint8_t* pkt);
  template <typename T_Msop>
  double calculateTimeYMD(const uint8_t* pkt);
  void sortBeamTable();

 protected:
  const LidarConstantParameter lidar_const_param_;
  RSDecoderParam param_;
  RSEchoMode echo_mode_;
  unsigned int pkts_per_frame_;
  unsigned int pkt_count_;
  unsigned int trigger_index_;
  unsigned int prev_angle_diff_;
  unsigned int rpm_;
  int start_angle_;
  int end_angle_;
  int cut_angle_;
  int last_azimuth_;
  bool angle_flag_;
  bool difop_flag_;
  float fov_time_jump_diff_;
  float time_duration_between_blocks_;
  float current_temperature_;
  float azi_diff_between_block_theoretical_;
  std::vector<int> vert_angle_list_;
  std::vector<int> hori_angle_list_;
  std::vector<std::size_t> beam_ring_table_;
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  static std::vector<double> cos_lookup_table_;
  static std::vector<double> sin_lookup_table_;
  std::function<double(const uint8_t*)> get_point_time_func_;
  std::function<void(const int&, const uint8_t*)> check_camera_trigger_func_;
};

template <typename T_Point>
inline DecoderBase<T_Point>::DecoderBase(
    const RSDecoderParam& param,
    const LidarConstantParameter& lidar_const_param)
    : lidar_const_param_(lidar_const_param),
      param_(param),
      echo_mode_(ECHO_STRONGEST),
      pkts_per_frame_(lidar_const_param.PKT_RATE / 10),
      pkt_count_(0),
      trigger_index_(0),
      prev_angle_diff_(RS_ONE_ROUND),
      rpm_(600),
      start_angle_(param.start_angle * 100),
      end_angle_(param.end_angle * 100),
      cut_angle_(param.cut_angle * 100),
      last_azimuth_(-36001),
      angle_flag_(true),
      difop_flag_(false),
      fov_time_jump_diff_(0),
      time_duration_between_blocks_(0),
      current_temperature_(0),
      azi_diff_between_block_theoretical_(20) {
  if (cut_angle_ > RS_ONE_ROUND) {
    cut_angle_ = 0;
  }
  if (this->start_angle_ > RS_ONE_ROUND || this->start_angle_ < 0 ||
      this->end_angle_ > RS_ONE_ROUND || this->end_angle_ < 0) {
    this->start_angle_ = 0;
    this->end_angle_ = RS_ONE_ROUND;
  }
  if (this->start_angle_ > this->end_angle_) {
    this->angle_flag_ = false;
  }

  /* Point time function*/
  if (this->param_.use_lidar_clock) {
    get_point_time_func_ = [this](const uint8_t* pkt) {
      return getLidarTime(pkt) * kSecondToNanoFactor;
    };
  } else {
    get_point_time_func_ = [this](const uint8_t* pkt) {
      double ret_time = (cyber::Time().Now().ToSecond() -
                         (this->lidar_const_param_.BLOCKS_PER_PKT - 1) *
                             this->time_duration_between_blocks_) *
                        kSecondToNanoFactor;
      return ret_time;
    };
  }
}

template <typename T_Point>
inline RSDecoderResult DecoderBase<T_Point>::processDifopPkt(
    const uint8_t* pkt) {
  if (pkt == NULL) {
    return PKT_NULL;
  }
  return decodeDifopPkt(pkt);
}

template <typename T_Point>
inline RSDecoderResult DecoderBase<T_Point>::processMsopPkt(
    const uint8_t* pkt, std::shared_ptr<std::vector<T_Point>> point_cloud_vec,
    std::shared_ptr<int> height) {
  if (pkt == NULL) {
    return PKT_NULL;
  }
  std::shared_ptr<int> azimuth = std::make_shared<int>(0);
  RSDecoderResult ret = decodeMsopPkt(pkt, point_cloud_vec, height, azimuth);
  if (ret != RSDecoderResult::DECODE_OK) {
    return ret;
  }
  this->pkt_count_++;
  switch (this->param_.split_frame_mode) {
    case SplitFrameMode::SPLIT_BY_ANGLE:
      if (*azimuth < this->last_azimuth_) {
        this->last_azimuth_ -= RS_ONE_ROUND;
      }
      if (this->last_azimuth_ != -36001 &&
          this->last_azimuth_ < this->cut_angle_ &&
          *azimuth >= this->cut_angle_) {
        this->last_azimuth_ = *azimuth;
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      this->last_azimuth_ = *azimuth;
      break;
    case SplitFrameMode::SPLIT_BY_FIXED_PKTS:
      if (this->pkt_count_ >= this->pkts_per_frame_) {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
    case SplitFrameMode::SPLIT_BY_CUSTOM_PKTS:
      if (this->pkt_count_ >= this->param_.num_pkts_split) {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
    default:
      break;
  }
  return DECODE_OK;
}

template <typename T_Point>
inline void DecoderBase<T_Point>::regRecvCallback(
    const std::function<void(const CameraTrigger&)>& callback) {
  camera_trigger_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
inline double DecoderBase<T_Point>::getLidarTemperature() {
  return current_temperature_;
}

template <typename T_Point>
inline void DecoderBase<T_Point>::loadCalibrationFile(
    const std::string& angle_path) {
  std::string line_str;
  std::ifstream fd_angle(angle_path.c_str(), std::ios::in);
  if (fd_angle.is_open()) {
    unsigned int row_index = 0;
    while (std::getline(fd_angle, line_str)) {
      std::stringstream ss(line_str);
      std::string str;
      std::vector<std::string> vect_str;
      while (std::getline(ss, str, ',')) {
        vect_str.emplace_back(str);
      }
      try {
        this->vert_angle_list_[row_index] =
            std::stof(vect_str.at(0)) * 100;  // degree
        this->hori_angle_list_[row_index] =
            std::stof(vect_str.at(1)) * 100;  // degree
      } catch (...) {
        break;
      }
      row_index++;
      if (row_index >= this->lidar_const_param_.LASER_NUM) {
        this->sortBeamTable();
        break;
      }
    }
    fd_angle.close();
  }
}

template <typename T_Point>
inline void DecoderBase<T_Point>::checkTriggerAngle(const int& angle,
                                                    const double& timestamp) {
  std::map<double, std::string>::iterator iter =
      param_.trigger_param.trigger_map.begin();
  for (size_t i = 0; i < trigger_index_; i++) {
    iter++;
  }
  if (iter != param_.trigger_param.trigger_map.end()) {
    unsigned int angle_diff = std::abs(iter->first * 100 - angle);
    if (angle_diff < prev_angle_diff_) {
      prev_angle_diff_ = angle_diff;
      return;
    } else {
      trigger_index_++;
      prev_angle_diff_ = RS_ONE_ROUND;
      for (auto cb : camera_trigger_cb_vec_) {
        cb(std::make_pair(iter->second, timestamp));
      }
    }
  }
}

/* 16, 32, & BP */
template <typename T_Point>
inline float DecoderBase<T_Point>::computeTemperature(
    const uint16_t& temp_raw) {
  uint8_t neg_flag = (temp_raw >> 8) & 0x80;
  float msb = (temp_raw >> 8) & 0x7F;
  float lsb = (temp_raw & 0x00FF) >> 3;
  float temp;
  if (neg_flag == 0x80) {
    temp = -1 * (msb * 32 + lsb) * 0.0625f;
  } else {
    temp = (msb * 32 + lsb) * 0.0625f;
  }

  return temp;
}

/* 128 & 80 */
template <typename T_Point>
inline float DecoderBase<T_Point>::computeTemperature(
    const uint8_t& temp_low, const uint8_t& temp_high) {
  uint8_t neg_flag = temp_low & 0x80;
  float msb = temp_low & 0x7F;
  float lsb = temp_high >> 4;
  float temp;
  if (neg_flag == 0x80) {
    temp = -1 * (msb * 16 + lsb) * 0.0625f;
  } else {
    temp = (msb * 16 + lsb) * 0.0625f;
  }

  return temp;
}

template <typename T_Point>
inline int DecoderBase<T_Point>::azimuthCalibration(const float& azimuth,
                                                    const int& channel) {
  return (static_cast<int>(azimuth) + this->hori_angle_list_[channel] +
          RS_ONE_ROUND) %
         RS_ONE_ROUND;
}

template <typename T_Point>
template <typename T_Msop>
inline double DecoderBase<T_Point>::calculateTimeUTC(const uint8_t* pkt) {
  T_Msop* mpkt_ptr = const_cast<T_Msop*>(reinterpret_cast<const T_Msop*>(pkt));
  union u_ts {
    uint8_t data[8];
    uint64_t ts;
  } timestamp;
  timestamp.data[7] = 0;
  timestamp.data[6] = 0;
  timestamp.data[5] = mpkt_ptr->header.timestamp_utc.sec[0];
  timestamp.data[4] = mpkt_ptr->header.timestamp_utc.sec[1];
  timestamp.data[3] = mpkt_ptr->header.timestamp_utc.sec[2];
  timestamp.data[2] = mpkt_ptr->header.timestamp_utc.sec[3];
  timestamp.data[1] = mpkt_ptr->header.timestamp_utc.sec[4];
  timestamp.data[0] = mpkt_ptr->header.timestamp_utc.sec[5];
  return static_cast<double>(timestamp.ts) +
         (static_cast<double>(
             RS_SWAP_LONG(mpkt_ptr->header.timestamp_utc.ns))) /
             1000000000.0d;
}

template <typename T_Point>
template <typename T_Msop>
inline double DecoderBase<T_Point>::calculateTimeYMD(const uint8_t* pkt) {
  T_Msop* mpkt_ptr = const_cast<T_Msop*>(reinterpret_cast<const T_Msop*>(pkt));
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
             1000000.0 -
         __timezone;
}

template <typename T_Point>
inline void DecoderBase<T_Point>::sortBeamTable() {
  std::vector<size_t> sorted_idx(this->lidar_const_param_.LASER_NUM);
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::sort(sorted_idx.begin(), sorted_idx.end(),
            [this](std::size_t i1, std::size_t i2) -> bool {
              return this->vert_angle_list_[i1] < this->vert_angle_list_[i2];
            });
  for (size_t i = 0; i < this->lidar_const_param_.LASER_NUM; i++) {
    this->beam_ring_table_[sorted_idx[i]] = i;
  }
}

inline const std::vector<double> initTrigonometricLookupTable(
    const std::function<double(const double)>& func) {
  std::vector<double> temp_table = std::vector<double>(RS_ONE_ROUND, 0.0);
  for (size_t i = 0; i < RS_ONE_ROUND; i++) {
    const double rad = RS_TO_RADS(static_cast<double>(i) / 100.0);
    temp_table[i] = func(rad);
  }
  return temp_table;
}

template <typename T_Point>
std::vector<double> DecoderBase<T_Point>::cos_lookup_table_ =
    initTrigonometricLookupTable([](const double rad) -> double {
      return std::cos(rad);
    });
template <typename T_Point>
std::vector<double> DecoderBase<T_Point>::sin_lookup_table_ =
    initTrigonometricLookupTable([](const double rad) -> double {
      return std::sin(rad);
    });
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
