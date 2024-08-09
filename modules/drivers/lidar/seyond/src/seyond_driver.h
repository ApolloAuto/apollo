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

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/seyond/proto/seyond.pb.h"

#include "seyond/sdk_common/inno_lidar_api.h"
#include "seyond/sdk_common/inno_lidar_other_api.h"
#include "seyond/sdk_common/inno_lidar_packet.h"
#include "seyond/sdk_common/inno_lidar_packet_utils.h"

namespace apollo {
namespace drivers {
namespace lidar {

using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;

struct SeyondParam {
  std::string device_ip = "";
  uint32_t port;
  uint32_t udp_port;
  bool reflectance_mode;
  uint32_t multiple_return;
  uint32_t coordinate_mode;
  double max_range;
  double min_range;
  std::string log_level = "";
  bool raw_packets_mode = false;

  void print() const {
    inno_log_info("------------------------------------------------------");
    inno_log_info("             Seyond Lidar Parameters ");
    inno_log_info("device_ip: %s", device_ip.c_str());
    inno_log_info("port: %d", port);
    inno_log_info("udp_port: %d", udp_port);
    inno_log_info("reflectance_mode: %d", reflectance_mode);
    inno_log_info("multiple_return: %d", multiple_return);
    inno_log_info("coordinate_mode: %d", coordinate_mode);
    inno_log_info("max_range: %f", max_range);
    inno_log_info("min_range: %f", min_range);
    inno_log_info("log_level: %s", log_level.c_str());
    inno_log_info("raw_packets_mode: %d", raw_packets_mode);
    inno_log_info("------------------------------------------------------");
  }
};

class SeyondDriver {
 public:
  SeyondDriver();
  ~SeyondDriver();

  static int32_t data_callback_s_(int32_t handle_, void *ctx,
                                  const InnoDataPacket *pkt) {
    return (reinterpret_cast<SeyondDriver *>(ctx))->data_callback_(pkt);
  }

  static void message_callback_s_(int32_t handle_, void *ctx,
                                  uint32_t from_remote,
                                  enum InnoMessageLevel level,
                                  enum InnoMessageCode code,
                                  const char *error_message) {
    (reinterpret_cast<SeyondDriver *>(ctx))
        ->message_callback_(from_remote, level, code, error_message);
  }

  static int32_t status_callback_s_(int32_t handle_, void *ctx,
                                    const InnoStatusPacket *pkt) {
    return (reinterpret_cast<SeyondDriver *>(ctx))->status_callback_(pkt);
  }

  static void log_callback_s_(void *ctx, enum InnoLogLevel level,
                              const char *header1, const char *header2,
                              const char *msg) {
    SeyondDriver::log_cb_s_(static_cast<int32_t>(level), header1, msg);
  }

  // lidar configuration
  void register_publish_packet_callback(
      const std::function<void(const InnoDataPacket *, bool)>
          &callback) {
    packet_publish_cb_ = callback;
  }
  void register_publish_point_callback(
      const std::function<void(std::shared_ptr<PointCloud>)> &cloud_callback,
      const std::function<std::shared_ptr<PointCloud>()> &allocate_callback) {
    cloud_publish_cb_ = cloud_callback;
    allocate_cloud_cb_ = allocate_callback;
    point_cloud_ptr_ = allocate_cloud_cb_();
  }
  void register_log_callback(
      const std::function<void(int32_t, const char *, const char *)>
          &log_callback) {
    log_cb_s_ = log_callback;
  }
  bool setup_lidar();
  bool init(SeyondParam& param);
  bool start();
  bool pause();
  bool stop();

  // callback group
  int32_t data_callback_(const InnoDataPacket *pkt);
  void message_callback_(uint32_t from_remote, enum InnoMessageLevel level,
                         enum InnoMessageCode code, const char *msg);
  int32_t status_callback_(const InnoStatusPacket *pkt);

  void set_log_and_message_level();

  int32_t process_scan_packet_(
      const std::shared_ptr<seyond::SeyondScan> &lidar_packets);
  void convert_and_parse_(const InnoDataPacket *pkt);
  int32_t process_data_packet_(const InnoDataPacket *pkt);
  template <typename PointType>
  void process_xyz_point_data_(bool is_en_data, bool is_use_refl,
                               uint32_t point_num, PointType point_ptr);

 public:
  // for generic
  bool anglehv_table_init_{false};
  std::vector<char> anglehv_table_;

 private:
  // apollo
  std::shared_ptr<PointCloud> point_cloud_ptr_{nullptr};

  std::function<void(const InnoDataPacket *, bool)> packet_publish_cb_;
  std::function<void(std::shared_ptr<PointCloud>)> cloud_publish_cb_;
  std::function<std::shared_ptr<PointCloud>()> allocate_cloud_cb_;

  static std::function<void(int32_t, const char*, const char*)> log_cb_s_;

  // config
  SeyondParam param_;
  int32_t handle_{-1};
  std::string lidar_name_{"seyond_lidar"};

  // frame status
  uint32_t packets_width_{0};
  int64_t current_frame_id_{-1};
  double current_ts_start_;
  uint64_t frame_points_width_;
  std::vector<uint8_t> convert_buffer_;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
