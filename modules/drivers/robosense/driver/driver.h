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
#include <thread>

#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/robosense/decoder/decoder_16.hpp"
#include "modules/drivers/robosense/decoder/decoder_factory.hpp"
#include "modules/drivers/robosense/driver/utility.h"
#include "modules/drivers/robosense/input/input.h"
#include "modules/drivers/robosense/proto/config.pb.h"
#include "modules/drivers/robosense/proto/robosense.pb.h"
#define PKT_DATA_LENGTH 1248
namespace apollo {
namespace drivers {
namespace robosense {
using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::robosense::RobosenseScan;
struct alignas(16) LidarPacketMsg {
  double timestamp = 0.0;
  std::string frame_id = "";
  std::array<uint8_t, PKT_DATA_LENGTH> packet{};  ///< lidar single packet
};

class RobosenseDriver {
 public:
  RobosenseDriver(const std::shared_ptr<Node> &node, const Config &conf)
      : node_(node), conf_(conf) {}
  ~RobosenseDriver() { stop(); }
  bool init();

 private:
  void getPackets();
  void processMsopPackets();
  void processDifopPackets();

 private:
  std::shared_ptr<Node> node_ = nullptr;
  Config conf_;
  std::shared_ptr<Writer<RobosenseScan>> scan_writer_ = nullptr;
  std::shared_ptr<Writer<PointCloud>> pointcloud_writer_ = nullptr;

 private:
  void prepareLidarScanMsg(std::shared_ptr<RobosenseScan> &msg);
  void preparePointsMsg(std::shared_ptr<PointCloud> &msg);
  void stop() {
    msop_pkt_queue_.clear();
    difop_pkt_queue_.clear();
    if (thread_flag_ == true) {
      thread_flag_ = false;
      lidar_thread_ptr_->join();
    }
  }

 private:
  Queue<LidarPacketMsg> msop_pkt_queue_;
  Queue<LidarPacketMsg> difop_pkt_queue_;
  bool thread_flag_;
  std::shared_ptr<std::thread> lidar_thread_ptr_;
  std::shared_ptr<DecoderBase<PointXYZIT>> lidar_decoder_ptr_;
  std::shared_ptr<Input> lidar_input_ptr_;
  uint32_t scan_seq_;
  uint32_t points_seq_;
  std::shared_ptr<PointCloud> point_cloud_ptr_;
  std::shared_ptr<RobosenseScan> scan_ptr_;
};
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo