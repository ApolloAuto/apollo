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

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

typedef struct {
  /*俯仰角*/
  float Pitch_deg;
  /*横滚角*/
  float Roll_deg;
  /*航向角*/
  float Yaw_deg;

  float GyroX;
  float GyroY;
  float GyroZ;
  float AccX_g; /* 零偏修正后加速度计 */
  float AccY_g; /* 零偏修正后加速度计 */
  float AccZ_g; /* 零偏修正后加速度计 */

  double Lon_gnss_deg;  // RTK融合后Longitude (deg)
  double Lat_gnss_deg;  // RTK融合后Latitude (deg)
  float Alt_gnss_m;     // RTK融合后Altitude (m)

  double Lon_deg;  // IMU融合后Longitude (deg)
  double Lat_deg;  // IMU融合后Latitude (deg)
  float Alt_m;     // IMU融合后Altitude (m)

  float VelN_mps; /*融合后 NED north velocity (m/s) */
  float VelE_mps; /*融合后 NED east velocity (m/s) */
  float VelD_mps; /*融合后 NED down velocity (m/s) */

  uint8_t InsStatus; /*初始化标志位*/

  uint8_t ModeStatus;

  uint8_t LoopType;
  float data1;
  float data2;
  float data3;

  int64_t SysTime_ms; /*时间戳 系统时间*/

  uint8_t flag_pos;

  // rtk age
  float differential_age;
} InsType;

template <typename T>
T toValue(const uint8_t* add, int& index) {
  T result = *reinterpret_cast<const T*>(add + index);
  index += sizeof(T);
  return result;
}

template <typename T>
T toValue(const char* add, int& index) {
  T result = *reinterpret_cast<const T*>(add + index);
  index += sizeof(T);
  return result;
}

class ProtocolAsensing {
 public:
  ProtocolAsensing();
  virtual ~ProtocolAsensing();

  /* subclass parse Protocol */
  virtual void subData(const uint8_t* sub_address, int& index) {
    printf("get protocol type:%02X %02X %02X \n", sub_address[index],
           sub_address[index + 1], sub_address[index + 2]);
  }

  void addData(const std::string& data);
  void clearCache();
  bool registProtocol(const std::string& protocolFlag, int length,
                      ProtocolAsensing* sub);
  void toQuaternion(double* rpy, double* quaterArray);

  inline int getLength(const std::string& flag) {
    return protocolLengthMap[flag];
  }
  inline uint32_t getDataSize() { return receive_data_.size(); }
  inline void changeLength(const std::string& flag, int len) {
    protocolLengthMap[flag] = len;
  }

  std::string getProtocol() { return cur_protocol_; }

 private:
  bool analysisData(const std::string& data);

 private:
  std::string receive_data_;  // 解析自定义数据
  std::string cur_protocol_;

  static std::map<std::string, int> protocolLengthMap;
  static std::map<std::string, ProtocolAsensing*> protocolMap;
};

class Decode_0A final : public ProtocolAsensing {
 public:
  explicit Decode_0A();
  virtual ~Decode_0A();

  void subData(const uint8_t* sub_address, int& index);

 private:
  /* protocol info */
  std::string m_type = "BDDB0A";
  int m_length = 34;

  std::string filename1;
  std::string content1;
};

class Decode_0B final : public ProtocolAsensing {
 public:
  explicit Decode_0B();
  virtual ~Decode_0B();

  void subData(const uint8_t* sub_address, int& index);
  void parse0B(const uint8_t* data, int& pos);
  void parseGnss(const uint8_t* data, int& pos);

  InsType insdata;
  uint8_t update_ins = 0;

  /* protocol info */
  std::string m_typeImu = "BDDB0B";
  int m_lengthImu = 63;
  std::string m_typeGnss = "BDDB10";
  int m_lengthGnss = 83;

 private:
  std::string filename2;
  std::string content2;

  std::string filename0b;
  std::string content0b;
};

bool createFileAndWrite(const std::string& filename,
                        const std::string& content);
bool AppendCsv(const std::string& filename,
               const std::vector<std::string>& data);
