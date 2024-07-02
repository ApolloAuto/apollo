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

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/seyond/proto/seyond.pb.h"
#include "modules/drivers/lidar/seyond/proto/seyond_config.pb.h"
#include "seyond/sdk_common/inno_lidar_api.h"
#include "seyond/sdk_common/inno_lidar_other_api.h"
#include "seyond/sdk_common/inno_lidar_packet.h"
#include "seyond/sdk_common/inno_lidar_packet_utils.h"

namespace apollo {
namespace drivers {
namespace seyond {

using apollo::cyber::Node;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::seyond::SeyondPacket;
using apollo::drivers::seyond::SeyondScan;

class SeyondDriver {
 public:
  explicit SeyondDriver(const std::shared_ptr<apollo::cyber::Node> &node,
                        const apollo::drivers::seyond::Config &seyond_conf);
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
    (reinterpret_cast<SeyondDriver *>(ctx))->log_callback_(level, header2, msg);
  }

  static double apollo_get_time_in_second_s(void *) {
    apollo::cyber::Time current_time = apollo::cyber::Time::Now();
    double ret = current_time.ToSecond();
    return ret;
  }

  // lidar configuration
  void setup_param();
  bool init_lidar();
  int32_t set_input_parameter_();
  bool init();
  bool start();
  bool pause();
  bool stop();

  // callback group
  int32_t data_callback_(const InnoDataPacket *pkt);
  void message_callback_(uint32_t from_remote, enum InnoMessageLevel level,
                         enum InnoMessageCode code, const char *msg);
  int32_t status_callback_(const InnoStatusPacket *pkt);

  void set_log_and_message_level();
  void log_callback_(enum InnoLogLevel level, const char *header2,
                     const char *msg);

  int32_t process_packets_and_publish_frame_(const InnoDataPacket *pkt);
  int32_t collect_packets_and_publish_packets_(const InnoDataPacket *pkt);
  int32_t process_scan_packet_(
      const std::shared_ptr<SeyondScan> &lidar_packets);
  void convert_and_parse_(const InnoDataPacket *pkt);
  int32_t process_data_packet_(const InnoDataPacket *pkt);
  template <typename PointType>
  void process_xyz_point_data_(bool is_en_data, bool is_use_refl,
                               uint32_t point_num, PointType point_ptr);

 private:
  // apollo
  std::shared_ptr<::apollo::cyber::Node> node_ptr_{nullptr};
  apollo::drivers::seyond::Config conf_;
  std::string pointcloud_channel_{"/apollo/sensor/seyond/PointCloud2"};
  std::string scan_channel_{"/apollo/sensor/seyond/Scan"};
  std::shared_ptr<Writer<SeyondScan>> scan_writer_ptr_{nullptr};
  std::shared_ptr<Reader<SeyondScan>> scan_reader_ptr_{nullptr};
  std::shared_ptr<Writer<PointCloud>> pcd_writer_ptr_{nullptr};
  std::shared_ptr<SeyondScan> scan_packets_ptr_{nullptr};
  std::shared_ptr<PointCloud> point_cloud_ptr_{nullptr};

  // config
  int32_t handle_{-1};
  std::string lidar_name_{"seyond"};
  std::string frame_id_{"seyond"};
  std::string device_ip_{"172.168.1.10"};
  uint32_t port_{8010};
  bool reflectance_mode_{false};
  uint32_t multiple_return_{1};
  uint32_t coordinate_mode_{3};
  bool direct_mode_{false};
  bool raw_packets_mode_{false};
  uint32_t aggregate_packets_num_{50};

  double max_range_{2000.0};
  double min_range_{0.4};

  std::string log_level_{"info"};

  int32_t udp_port_{8010};

  // for robinw-generic
  uint32_t table_send_hz_{10};
  uint32_t frame_count_{0};
  bool anglehv_table_init_{false};
  std::vector<char> anglehv_table_;

  // frame status
  uint32_t packets_width_{0};
  int64_t current_frame_id_{-1};
  double current_ts_start_;
  uint64_t frame_points_width_;
  std::vector<uint8_t> convert_buffer_;
};

}  // namespace seyond
}  // namespace drivers
}  // namespace apollo
