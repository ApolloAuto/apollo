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

namespace apollo {
namespace v2x {

using apollo::localization::LocalizationEstimate;

V2xProxy::V2xProxy()
    : node_(cyber::CreateNode("v2x_proxy")), init_flag_(false) {
  if (node_ == nullptr) {
    AFATAL << "Create v2x proxy node failed.";
    exit(1);
  }

  auto x2v_trafficlight_timer_period = ceil(
      (1.0 / static_cast<int>(FLAGS_x2v_trafficlight_timer_frequency)) * 1000);
  x2v_trafficlight_timer_.reset(
      new cyber::Timer(static_cast<uint32_t>(x2v_trafficlight_timer_period),
                       [this]() { this->OnX2vTrafficLightTimer(); }, false));
  auto v2x_carstatus_timer_period = ceil(
      (1.0 / static_cast<int>(FLAGS_v2x_carstatus_timer_frequency)) * 1000);
  v2x_carstatus_timer_.reset(
      new cyber::Timer(static_cast<uint32_t>(v2x_carstatus_timer_period),
                       [this]() { this->OnV2xCarStatusTimer(); }, false));

  os_interface_.reset(new OsInterFace());
  obu_interface_.reset(new ObuInterFaceGrpcImpl());

  x2v_trafficlight_ = std::make_shared<IntersectionTrafficLightData>();
  v2x_carstatus_ = std::make_shared<CarStatus>();

  if (!x2v_trafficlight_timer_ || !v2x_carstatus_timer_ || !os_interface_ ||
      !obu_interface_ || !x2v_trafficlight_ || !v2x_carstatus_) {
    AFATAL << "Create timer or interface failed.";
    exit(1);
  }

  hdmap_.reset(new apollo::hdmap::HDMap());
  std::string map_name = FLAGS_map_dir + "/" + FLAGS_base_map_filename;
  if (FLAGS_debug_flag) {
    map_name = FLAGS_hdmap_file_name;
  }
  if (hdmap_->LoadMapFromFile(map_name) != 0) {
    AFATAL << "Failed to load hadmap file: " << FLAGS_hdmap_file_name;
    return;
  }
  AINFO << "load hdmap file: " << FLAGS_hdmap_file_name;

  if (!os_interface_->InitFlag() || !obu_interface_->InitFlag()) {
    AFATAL << "Failed to init os interface or obu interface";
    return;
  }
  first_flag_reader_ = node_->CreateReader<StatusResponse>(
      "/apollo/v2x/inner/sync_flag",
      [this](const std::shared_ptr<const StatusResponse>& msg) {
        x2v_trafficlight_timer_->Start();
      });
  if (first_flag_reader_ == nullptr) {
    AERROR << "Create sync flag reader failed";
    exit(1);
  }
  // x2v_trafficlight_timer_->Start();
  v2x_carstatus_timer_->Start();

  init_flag_ = true;
}

bool V2xProxy::InitFlag() { return init_flag_; }

bool V2xProxy::TrafficLightProc(CurrentLaneTrafficLight* msg) {
  if (!msg->has_gps_x_m() || !msg->has_gps_y_m()) {
    AERROR << "Error::v2x trafficlight ignore, gps point is null";
    return false;
  }
  apollo::common::PointENU point;
  point.set_x(msg->gps_x_m());
  point.set_y(msg->gps_y_m());
  std::vector<apollo::hdmap::SignalInfoConstPtr> signals;
  if (hdmap_->GetForwardNearestSignalsOnLane(point, 1000.0, &signals) != 0) {
    AERROR << "Error::v2x trafficlight ignore, hdmap get no signals";
    AERROR << "traffic light size : " << signals.size();
    return false;
  }
  for (auto i = signals.begin(); i != signals.end(); i++) {
    if ((*i)->id().id().empty()) {
      AERROR << "Error::v2x trafficlight ignore, signals id is empty";
      return false;
    }
  }
  if (signals.size() == 1) {
    auto single = msg->mutable_single_traffic_light(0);
    single->set_id(signals[0]->id().id());
    return true;
  }
  auto color = msg->single_traffic_light(0).color();
  msg->clear_single_traffic_light();

  for (auto i = signals.begin(); i != signals.end(); i++) {
    auto single = msg->add_single_traffic_light();
    single->set_id((*i)->id().id());
    single->set_color(color);
  }
  return true;
}

void V2xProxy::OnX2vTrafficLightTimer() {
  x2v_trafficlight_->Clear();
  obu_interface_->GetV2xTrafficLightFromObu(x2v_trafficlight_);
  if (!x2v_trafficlight_->has_current_lane_trafficlight()) {
    AERROR << "Error:v2x trafficlight ignore, no traffic light contained.";
    return;
  }
  auto current_traff = x2v_trafficlight_->mutable_current_lane_trafficlight();
  if (current_traff->single_traffic_light().empty()) {
    AERROR << "Error:v2x trafficlight ignore, no traffic light contained.";
    return;
  }
  ADEBUG << x2v_trafficlight_->DebugString();
  if (!TrafficLightProc(current_traff)) {
    return;
  }
  os_interface_->SendV2xTrafficLightToOs(x2v_trafficlight_);
}

void V2xProxy::OnV2xCarStatusTimer() {
  v2x_carstatus_->Clear();
  auto localization = std::make_shared<LocalizationEstimate>();
  os_interface_->GetLocalizationFromOs(localization);
  if (!localization || !localization->has_header() ||
      !localization->has_pose()) {
    AERROR << "Error:localization ignore, no pose or header in it.";
    return;
  }
  v2x_carstatus_->mutable_localization()->CopyFrom(*localization);
  obu_interface_->SendCarStatusToObu(v2x_carstatus_);
}

}  // namespace v2x
}  // namespace apollo
