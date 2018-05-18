/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_DRIVERS_GNSS_RAW_STREAM_H_
#define MODULES_DRIVERS_GNSS_RAW_STREAM_H_

#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "ros/include/ros/ros.h"
#include "ros/include/std_msgs/String.h"

#include "modules/drivers/gnss/parser/data_parser.h"
#include "modules/drivers/gnss/parser/rtcm_parser.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

using apollo::drivers::gnss_status::StreamStatus;
using apollo::drivers::gnss_status::StreamStatus_Type;

class RawStream {
 public:
  explicit RawStream(const config::Config& config);
  ~RawStream();
  bool Init();

  struct Status {
    bool filter[Stream::NUM_STATUS] = {false};
    Stream::Status status;
  };

  void Start();

 private:
  void DataSpin();
  void RtkSpin();
  bool Connect();
  bool Disconnect();
  bool Login();
  bool Logout();
  void StreamStatusCheck();
  void PublishRtkData(size_t length);
  void PushGpgga(size_t length);
  void GpsbinSpin();
  void GpsbinCallback(const std_msgs::String& raw_data);
  void OnWheelVelocityTimer(const ros::TimerEvent&);

  ros::Timer wheel_velocity_timer_;
  static constexpr size_t BUFFER_SIZE = 2048;
  uint8_t buffer_[BUFFER_SIZE] = {0};
  uint8_t buffer_rtk_[BUFFER_SIZE] = {0};

  std::shared_ptr<Stream> data_stream_;
  std::shared_ptr<Stream> command_stream_;
  std::shared_ptr<Stream> in_rtk_stream_;
  std::shared_ptr<Stream> out_rtk_stream_;

  std::shared_ptr<Status> data_stream_status_;
  std::shared_ptr<Status> command_stream_status_;
  std::shared_ptr<Status> in_rtk_stream_status_;
  std::shared_ptr<Status> out_rtk_stream_status_;

  bool rtk_software_solution_ = false;
  bool push_location_ = false;
  bool is_healthy_ = true;
  config::Config config_;

  const std::string raw_data_topic_;
  const std::string rtcm_data_topic_;

  StreamStatus stream_status_;
  std::unique_ptr<std::thread> data_thread_ptr_;
  std::unique_ptr<std::thread> rtk_thread_ptr_;
  std::unique_ptr<DataParser> data_parser_ptr_;
  std::unique_ptr<RtcmParser> rtcm_parser_ptr_;
  std::unique_ptr<std::thread> gpsbin_thread_ptr_;
  std::unique_ptr<std::ofstream> gpsbin_stream_ = nullptr;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_RAW_STREAM_H_
