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
 * @file v2x_proxy.cc
 * @brief define v2x proxy class
 */
#include "modules/v2x/v2x_proxy/app/v2x_proxy.h"

#include <cmath>
#include <cstdio>
#include <ostream>
#include <vector>

#include "modules/v2x/common/v2x_proxy_gflags.h"

namespace apollo {
namespace v2x {
using ::apollo::canbus::ChassisDetail;
using ::apollo::v2x::CarStatus;
using ::apollo::v2x::StatusResponse;
using ::apollo::v2x::obu::ObuRsi;
using ::apollo::v2x::obu::ObuTrafficLight;

V2xProxy::~V2xProxy() {
  if (recv_thread_ != nullptr && recv_thread_->joinable()) {
    recv_thread_->join();
  }
  if (planning_thread_ != nullptr && planning_thread_->joinable()) {
    planning_thread_->join();
  }
  if (obs_thread_ != nullptr && obs_thread_->joinable()) {
    obs_thread_->join();
  }
}

V2xProxy::V2xProxy(std::shared_ptr<::apollo::hdmap::HDMap> hdmap)
    : node_(::apollo::cyber::CreateNode("v2x_proxy")), exit_(false) {
  if (node_ == nullptr) {
    AFATAL << "Create v2x proxy node failed.";
    exit(1);
  }
  internal_ = std::make_shared<InternalData>();
  hdmap_ = std::make_shared<::apollo::hdmap::HDMap>();
  const auto hdmap_file = apollo::hdmap::BaseMapFile();
  if (0 != hdmap_->LoadMapFromFile(hdmap_file)) {
    AERROR << "Failed to load hadmap file: " << hdmap_file;
    return;
  }
  ::apollo::cyber::TimerOption v2x_car_status_timer_option;
  v2x_car_status_timer_option.period =
      static_cast<uint32_t>((1000 + FLAGS_v2x_car_status_timer_frequency - 1) /
                            FLAGS_v2x_car_status_timer_frequency);
  v2x_car_status_timer_option.callback = [this]() {
    this->OnV2xCarStatusTimer();
  };
  v2x_car_status_timer_option.oneshot = false;
  v2x_car_status_timer_.reset(
      new ::apollo::cyber::Timer(v2x_car_status_timer_option));
  os_interface_.reset(new OsInterFace());
  obu_interface_.reset(new ObuInterFaceGrpcImpl());
  recv_thread_.reset(new std::thread([this]() {
    while (!exit_.load()) {
      this->RecvTrafficlight();
    }
  }));
  CHECK(!!hdmap_ && !!v2x_car_status_timer_);
  CHECK(!!os_interface_ && !!obu_interface_);
  CHECK(os_interface_->InitFlag() && obu_interface_->InitFlag());
  planning_thread_.reset(new std::thread([this]() {
    while (!exit_.load()) {
      this->RecvOsPlanning();
    }
  }));
  obs_thread_.reset(new std::thread([this]() {
    while (!exit_.load()) {
      std::shared_ptr<::apollo::v2x::V2XObstacles> obs = nullptr;
      this->obu_interface_->GetV2xObstaclesFromObu(&obs);  // Blocked
      this->os_interface_->SendV2xObstacles2Sys(obs);
    }
  }));
  v2x_car_status_timer_->Start();
  GetRsuListFromFile(FLAGS_rsu_whitelist_name, &rsu_list_);
  init_flag_ = true;
}

bool V2xProxy::GetRsuListFromFile(const std::string &filename,
                                  std::set<std::string> *whitelist) {
  if (nullptr == whitelist) {
    return false;
  }
  std::ifstream input_file(filename);
  if (!input_file) {
    return false;
  }
  std::string line;
  while (getline(input_file, line)) {
    whitelist->insert(line);
  }
  return !whitelist->empty();
}

bool V2xProxy::InitFlag() { return init_flag_; }

void V2xProxy::RecvOsPlanning() {
  auto adc_trajectory = std::make_shared<::apollo::planning::ADCTrajectory>();
  auto res_light =
      std::make_shared<::apollo::perception::TrafficLightDetection>();
  os_interface_->GetPlanningAdcFromOs(adc_trajectory);
  // OK get planning message
  std::shared_ptr<OSLight> last_os_light = nullptr;
  {
    std::lock_guard<std::mutex> lock(lock_last_os_light_);

    auto now_us = ::apollo::cyber::Time::MonoTime().ToMicrosecond();
    if (last_os_light_ == nullptr ||
        2000LL * 1000 * 1000 < now_us - ts_last_os_light_) {
      AWARN << "V2X Traffic Light is too old!";
      last_os_light_ = nullptr;
    } else {
      ADEBUG << "V2X Traffic Light is on time.";
      last_os_light = std::make_shared<OSLight>();
      last_os_light->CopyFrom(*last_os_light_);
    }
  }
  // proc planning message
  bool res_proc_planning_msg = internal_->ProcPlanningMessage(
      adc_trajectory.get(), last_os_light.get(), &res_light);
  if (!res_proc_planning_msg) {
    return;
  }
  os_interface_->SendV2xTrafficLight4Hmi2Sys(res_light);
}

void V2xProxy::RecvTrafficlight() {
  // get traffic light from obu
  std::shared_ptr<ObuLight> x2v_traffic_light = nullptr;
  obu_interface_->GetV2xTrafficLightFromObu(&x2v_traffic_light);
  os_interface_->SendV2xObuTrafficLightToOs(x2v_traffic_light);
  auto os_light = std::make_shared<OSLight>();
  std::string junction_id = "";
  {
    std::lock_guard<std::mutex> lg(lock_hdmap_junction_id_);
    junction_id = hdmap_junction_id_;
  }
  bool res_success_ProcTrafficlight = internal_->ProcTrafficlight(
      hdmap_, x2v_traffic_light.get(), junction_id, u_turn_,
      FLAGS_traffic_light_distance, FLAGS_check_time, &os_light);
  if (!res_success_ProcTrafficlight) {
    return;
  }
  utils::UniqueOslight(os_light.get());
  os_interface_->SendV2xTrafficLightToOs(os_light);
  // save for hmi
  std::lock_guard<std::mutex> lock(lock_last_os_light_);
  ts_last_os_light_ = ::apollo::cyber::Time::MonoTime().ToMicrosecond();
  last_os_light_ = os_light;
}

double cal_distance(const ::apollo::common::PointENU &p1,
                    const ::apollo::common::PointENU &p2) {
  double x = p1.x() - p2.x();
  double y = p1.y() - p2.y();
  return std::sqrt(x * x + y * y);
}

void V2xProxy::OnV2xCarStatusTimer() {
  // get loc
  auto localization =
      std::make_shared<::apollo::localization::LocalizationEstimate>();
  os_interface_->GetLocalizationFromOs(localization);
  if (nullptr == localization) {
    return;
  }
  std::set<std::string> rsu_whitelist;
  {
    std::lock_guard<std::mutex> lg(rsu_list_mutex_);
    rsu_whitelist.insert(rsu_list_.cbegin(), rsu_list_.cend());
  }
  ::apollo::common::PointENU car_position;
  car_position.set_x(localization->pose().position().x());
  car_position.set_y(localization->pose().position().y());
  std::shared_ptr<::apollo::v2x::CarStatus> v2x_car_status = nullptr;
  double heading = 0.0;
  std::string res_junction_id = "";
  bool ret_get_rsu_info = utils::GetRsuInfo(
      hdmap_, *localization, rsu_whitelist, FLAGS_traffic_light_distance,
      FLAGS_heading_difference, &v2x_car_status, &res_junction_id, &heading);
  {
    std::lock_guard<std::mutex> lg(lock_hdmap_junction_id_);
    hdmap_junction_id_ = res_junction_id;
  }
  if (!ret_get_rsu_info) {
    return;
  }
  // calc heading
  if (!init_heading_) {
    heading_ = heading;
    init_heading_ = true;
  }
  if (std::fabs(heading_ - heading) > 1.0) {
    u_turn_ = true;
  }
  obu_interface_->SendCarStatusToObu(v2x_car_status);
}

}  // namespace v2x
}  // namespace apollo
