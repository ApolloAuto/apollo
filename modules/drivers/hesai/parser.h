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

#ifndef LIDAR_HESAI_SRC_PARSE_H
#define LIDAR_HESAI_SRC_PARSE_H

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/hesai/const_var.h"
#include "modules/drivers/hesai/proto/config.pb.h"
#include "modules/drivers/hesai/proto/hesai.pb.h"
#include "modules/drivers/hesai/tcp_cmd_client.h"
#include "modules/drivers/hesai/type_defs.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace hesai {

inline double degreeToRadian(double degree) { return degree * PI / 180; }

using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::hesai::HesaiScan;

using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;

class Parser {
 public:
  Parser(const std::shared_ptr<Node>& node, const Config& conf);
  virtual ~Parser();
  void Parse(const uint8_t* data, int size, bool* is_end);
  bool Parse(const std::shared_ptr<HesaiScan>& scan);
  bool Init();

 private:
  std::thread online_calibration_thread_;
  void GetCalibrationThread();
  bool CheckIsEnd(bool is_end);
  void LoadCalibrationThread();
  bool LoadCalibration(const std::string& content);
  bool LoadCalibration(const char* path_file);
  void PublishRawPointCloud(int ret = -1);

  bool inited_ = false;
  void Stop();
  std::atomic<bool> running_ = {true};

 protected:
  virtual void ParseRawPacket(const uint8_t* buf, const int len,
                              bool* is_end) = 0;
  void CheckPktTime(double time_sec);
  void ResetRawPointCloud();

  bool is_calibration_ = false;
  std::shared_ptr<Node> node_;
  Config conf_;
  std::shared_ptr<Writer<PointCloud>> raw_pointcloud_writer_;
  int pool_size_ = 8;
  int pool_index_ = 0;
  uint64_t raw_last_time_ = 0;
  int seq_index_ = 0;
  std::deque<std::shared_ptr<PointCloud>> raw_pointcloud_pool_;
  // std::shared_ptr<CCObjectPool<PointCloud>> raw_pointcloud_pool_ = nullptr;
  std::shared_ptr<PointCloud> raw_pointcloud_out_ = nullptr;

  int last_azimuth_ = 0;
  int tz_second_ = 0;
  int start_angle_ = 0;
  uint32_t min_packets_ = HESAI40_MIN_PACKETS;
  uint32_t max_packets_ = HESAI40_MAX_PACKETS;
  uint32_t packet_nums_ = 0;

  double elev_angle_map_[LASER_COUNT_L64] = {0};
  double horizatal_azimuth_offset_map_[LASER_COUNT_L64] = {0};
};

/***********************hesai40P***********************/
class Hesai40Parser : public Parser {
 public:
  Hesai40Parser(const std::shared_ptr<Node>& node, const Config& conf);
  ~Hesai40Parser();

 protected:
  void ParseRawPacket(const uint8_t* buf, const int len, bool* is_end) override;

 private:
  void CalcPointXYZIT(Hesai40Packet* pkt, int blockid);
  double block_offset_[BLOCKS_PER_PACKET];
  double laser_offset_[LASER_COUNT];
};

/***********************hesai64***********************/
class Hesai64Parser : public Parser {
 public:
  Hesai64Parser(const std::shared_ptr<Node>& node, const Config& conf);
  ~Hesai64Parser();

 protected:
  void ParseRawPacket(const uint8_t* buf, const int len, bool* is_end) override;

 private:
  void CalcPointXYZIT(Hesai64Packet* pkt, int blockid, uint8_t chLaserNumber);
  double block_offset_[BLOCKS_PER_PACKET_L64] = {0};
  double laser_offset_[LASER_COUNT_L64] = {0};
};

class ParserFactory {
 public:
  static Parser* CreateParser(const std::shared_ptr<Node>& node,
                              const Config& conf);
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif
