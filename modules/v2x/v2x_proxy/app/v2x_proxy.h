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

#include <memory>
#include <string>
#include <vector>

#include "modules/common/configs/config_gflags.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/v2x/v2x_proxy/obu_interface/obu_interface_grpc_impl.h"
#include "modules/v2x/v2x_proxy/os_interface/os_interface.h"

namespace apollo {
namespace v2x {

class V2xProxy {
 public:
  V2xProxy();
  ~V2xProxy() {}
  bool InitFlag();

 private:
  std::unique_ptr<cyber::Timer> x2v_trafficlight_timer_;
  std::unique_ptr<cyber::Timer> v2x_carstatus_timer_;
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Reader<StatusResponse>> first_flag_reader_ = nullptr;
  std::unique_ptr<OsInterFace> os_interface_;
  std::unique_ptr<ObuInterFaceGrpcImpl> obu_interface_;

  std::shared_ptr<IntersectionTrafficLightData> x2v_trafficlight_;
  std::shared_ptr<CarStatus> v2x_carstatus_;

  /* function obu to car traffic light timer callback
   */
  void OnX2vTrafficLightTimer();

  /* function car to obu car status timer callback
   */
  void OnV2xCarStatusTimer();

  /* function trafficlight proc according to hdmap waiting for add
  @param output mutable CurrentLaneTrafficLight msg
  */
  bool TrafficLightProc(CurrentLaneTrafficLight *msg);
  std::unique_ptr<apollo::hdmap::HDMap> hdmap_;

  bool init_flag_ = false;
};

}  // namespace v2x
}  // namespace apollo
