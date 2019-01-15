/******************************************************************************
 * Copyright 2019 The CiDi Authors. All Rights Reserved.
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

#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"

#include "modules/drivers/cidiv2x/stream/parser/cidiv2x_parser.h"
#include "modules/drivers/cidiv2x/stream/proto/config.pb.h"
#include "modules/drivers/cidiv2x/stream/proto/stream_status.pb.h"
#include "modules/drivers/cidiv2x/stream/stream.h"

namespace apollo {
namespace drivers {
namespace cidiv2x {

using apollo::drivers::cidiv2x::stream_status::StreamStatus;
using apollo::drivers::cidiv2x::stream_status::StreamStatus_Type;

class RawStream {
 public:
  explicit RawStream(const config::Config& config,
                     const std::shared_ptr<apollo::cyber::Node>& node);
  ~RawStream();
  bool Init();

  struct Status {
    bool filter[Stream::NUM_STATUS] = {false};
    Stream::Status status;
  };

  void Start();
  void Stop();

 private:
  bool isRunning() const;
  void DataSpin();
  void HeartSpin();
  bool Connect();
  bool Disconnect();
  bool Login();
  bool Logout();
  void StreamStatusCheck();

  static constexpr size_t BUFFER_SIZE = 1024;
  uint8_t buffer_[BUFFER_SIZE] = {0};

  std::shared_ptr<Stream> data_stream_;
  std::shared_ptr<Stream> command_stream_;

  std::shared_ptr<Status> data_stream_status_;
  std::shared_ptr<Status> command_stream_status_;

  bool is_healthy_ = false;
  bool is_running_ = false;
  config::Config config_;

  StreamStatus stream_status_;
  std::unique_ptr<std::thread> data_thread_ptr_;
  std::unique_ptr<std::thread> heart_thread_ptr_;
  std::unique_ptr<CidiV2xParser> data_parser_ptr_;

  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
};

}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo
