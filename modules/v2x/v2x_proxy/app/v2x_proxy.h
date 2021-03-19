/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file v2x_proxy.h
 * @brief define v2x proxy class
 */

#pragma once

#include <atomic>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/v2x/v2x_proxy/app/utils.h"
#include "modules/v2x/v2x_proxy/obu_interface/obu_interface_grpc_impl.h"
#include "modules/v2x/v2x_proxy/os_interface/os_interface.h"
#include "modules/v2x/v2x_proxy/proto_adapter/proto_adapter.h"

namespace apollo {
namespace v2x {
bool IsRushHour();

class V2xProxy {
 private:
  std::shared_ptr<InternalData> internal_ = nullptr;
  std::string hdmap_junction_id_ = kUnknownJunctionId;
  std::mutex lock_hdmap_junction_id_;
  // for hmi traffic lights
  std::shared_ptr<OSLight> last_os_light_ = nullptr;
  uint64_t ts_last_os_light_ = 0;
  std::mutex lock_last_os_light_;

 public:
  explicit V2xProxy(std::shared_ptr<::apollo::hdmap::HDMap> hdmap = nullptr);

  ~V2xProxy();

  bool InitFlag();

  void stop() {
    exit_ = true;
    v2x_car_status_timer_->Stop();
    obu_status_timer_->Stop();
    rsu_whitelist_timer_->Stop();
  }

  bool GetRsuListFromFile(const std::string &filename,
                          std::set<std::string> *whitelist);

 private:
  /* function RecvTrafficlight, get traffic light msg from grpc with timeout
   */
  void RecvTrafficlight();

  void RecvOsPlanning();
  /* function obu to car traffic light timer callback
   */
  // void on_x2v_traffic_light_timer();
  std::unique_ptr<::apollo::cyber::Timer> v2x_car_status_timer_;
  std::unique_ptr<::apollo::cyber::Timer> obu_status_timer_;
  std::unique_ptr<::apollo::cyber::Timer> rsu_whitelist_timer_;

  std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
  std::unique_ptr<OsInterFace> os_interface_;
  std::unique_ptr<ObuInterFaceGrpcImpl> obu_interface_;
  std::unique_ptr<std::thread> recv_thread_ = nullptr;
  std::unique_ptr<std::thread> planning_thread_ = nullptr;
  std::unique_ptr<std::thread> rsi_thread_ = nullptr;
  std::unique_ptr<std::thread> obs_thread_ = nullptr;

  /* function car to obu car status timer callback
   */
  void OnV2xCarStatusTimer();

  std::shared_ptr<::apollo::hdmap::HDMap> hdmap_;

  bool init_flag_ = false;
  bool init_heading_ = false;
  double heading_ = 0.0001;
  bool u_turn_ = false;
  std::atomic<bool> exit_;
  std::mutex rsu_list_mutex_;
  std::set<std::string> rsu_list_;
};

}  // namespace v2x
}  // namespace apollo
