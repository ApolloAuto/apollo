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
 * @file os_interface.cc
 * @brief define v2x proxy module and apollo os interface
 */

#include "modules/v2x/v2x_proxy/os_interface/os_interface.h"

#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace v2x {

using ::apollo::localization::LocalizationEstimate;
using ::apollo::perception::PerceptionObstacles;
using ::apollo::planning::ADCTrajectory;
using ::apollo::v2x::IntersectionTrafficLightData;

OsInterFace::OsInterFace()
    : node_(::apollo::cyber::CreateNode("v2x_os_interface")),
      init_flag_(false) {
  init_flag_ = !!node_ && InitReaders() && InitWriters();
}

OsInterFace::~OsInterFace() {}

bool OsInterFace::InitReaders() {
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<const LocalizationEstimate> &msg) {
        std::lock_guard<std::mutex> lg(mutex_localization_);
        current_localization_.Clear();
        current_localization_.CopyFrom(*msg);
      });
  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<const ADCTrajectory> &msg) {
        {
          std::lock_guard<std::mutex> lg(mutex_planning_);
          adc_trajectory_msg_.Clear();
          adc_trajectory_msg_.CopyFrom(*msg);
          flag_planning_new_ = true;
        }
        cond_planning_.notify_one();
      });
  return nullptr != localization_reader_ && nullptr != planning_reader_;
}

bool OsInterFace::InitWriters() {
  v2x_obu_traffic_light_writer_ =
      node_->CreateWriter<::apollo::v2x::obu::ObuTrafficLight>(
          FLAGS_v2x_obu_traffic_light_topic);
  v2x_traffic_light_writer_ =
      node_->CreateWriter<::apollo::v2x::IntersectionTrafficLightData>(
          FLAGS_v2x_traffic_light_topic);
  v2x_traffic_light_hmi_writer_ =
      node_->CreateWriter<::apollo::perception::TrafficLightDetection>(
          FLAGS_v2x_traffic_light_for_hmi_topic);
  v2x_obstacles_internal_writer_ =
      node_->CreateWriter<apollo::v2x::V2XObstacles>(
          FLAGS_v2x_internal_obstacle_topic);
  return nullptr != v2x_obu_traffic_light_writer_ &&
         nullptr != v2x_traffic_light_hmi_writer_ &&
         nullptr != v2x_obstacles_internal_writer_ &&
         nullptr != v2x_traffic_light_writer_;
}

void OsInterFace::GetLocalizationFromOs(
    const std::shared_ptr<LocalizationEstimate> &msg) {
  AINFO << "get localization result from os";
  std::lock_guard<std::mutex> lg(mutex_localization_);
  msg->CopyFrom(current_localization_);
}

void OsInterFace::GetPlanningAdcFromOs(
    const std::shared_ptr<::apollo::planning::ADCTrajectory> &msg) {
  AINFO << "get planning adc from os";
  std::unique_lock<std::mutex> lg(mutex_planning_);
  cond_planning_.wait(lg, [this]() { return flag_planning_new_; });
  flag_planning_new_ = false;
  msg->CopyFrom(adc_trajectory_msg_);
}

void OsInterFace::SendV2xObuTrafficLightToOs(
    const std::shared_ptr<::apollo::v2x::obu::ObuTrafficLight> &msg) {
  if (nullptr == msg) {
    return;
  }
  AINFO << "send v2x obu traffic_light to os";
  SendMsgToOs(v2x_obu_traffic_light_writer_.get(), msg);
  AINFO << "v2x obu traffic_light result: " << msg->DebugString();
}

void OsInterFace::SendV2xObstacles2Sys(
    const std::shared_ptr<apollo::v2x::V2XObstacles> &msg) {
  if (nullptr == msg) {
    return;
  }
  AINFO << "send v2x obu traffic_light to os";
  SendMsgToOs(v2x_obstacles_internal_writer_.get(), msg);
  AINFO << "v2x obu traffic_light result: " << msg->DebugString();
}

void OsInterFace::SendV2xTrafficLightToOs(
    const std::shared_ptr<IntersectionTrafficLightData> &msg) {
  if (nullptr == msg) {
    return;
  }
  AINFO << "send v2x traffic_light to os";
  SendMsgToOs(v2x_traffic_light_writer_.get(), msg);
  AINFO << "v2x traffic_light result: " << msg->DebugString();
}

void OsInterFace::SendV2xTrafficLight4Hmi2Sys(
    const std::shared_ptr<::apollo::perception::TrafficLightDetection> &msg) {
  if (nullptr == msg) {
    return;
  }
  AINFO << "send v2x tl4hmi to os";
  SendMsgToOs(v2x_traffic_light_hmi_writer_.get(), msg);
  AINFO << "v2x tl4hmi result: " << msg->DebugString();
}

}  // namespace v2x
}  // namespace apollo
