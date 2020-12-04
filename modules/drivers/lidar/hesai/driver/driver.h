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

#ifndef LIDAR_HESAI_DRIVER_H_
#define LIDAR_HESAI_DRIVER_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/lidar/hesai/parser/parser.h"
#include "modules/drivers/lidar/hesai/input/udp_input.h"

namespace apollo {
namespace drivers {
namespace hesai {

class HesaiDriver {
 public:
  HesaiDriver(const std::shared_ptr<::apollo::cyber::Node>& node,
              const Config& conf, const std::shared_ptr<Parser>& parser)
      : node_(node), conf_(conf), parser_(parser) {}
  ~HesaiDriver() { Stop(); }
  bool Init();

 private:
  std::shared_ptr<::apollo::cyber::Node> node_ = nullptr;
  Config conf_;
  std::shared_ptr<Parser> parser_ = nullptr;
  std::shared_ptr<Input> input_ = nullptr;
  std::shared_ptr<::apollo::cyber::Writer<HesaiScan>> scan_writer_ = nullptr;
  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;
  std::thread poll_thread_;
  std::thread process_thread_;
  std::atomic<bool> running_ = {true};
  std::deque<std::shared_ptr<HesaiScan>> scan_buffer_;
  int scan_buffer_size_ = 10;
  int index_ = 0;
  int tz_second_ = 0;
  std::vector<std::shared_ptr<HesaiPacket>> pkt_buffer_;
  int pkt_index_ = 0;
  int pkt_buffer_capacity_ = 0;
  std::list<std::shared_ptr<HesaiPacket>> pkt_queue_;

 private:
  void PollThread();
  void ProcessThread();
  void ProcessGps(const HesaiPacket& pkt);

  void Stop() {
    AINFO << "driver stoping...";
    running_.store(false);
    packet_condition_.notify_all();
    if (poll_thread_.joinable()) {
      poll_thread_.join();
    }
    if (process_thread_.joinable()) {
      process_thread_.join();
    }
  }
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif
