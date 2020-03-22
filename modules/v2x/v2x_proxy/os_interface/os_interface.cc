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

namespace apollo {
namespace v2x {

using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;

OsInterFace::OsInterFace()
    : node_(cyber::CreateNode("v2x_os_interface")), init_flag_(false) {
  ACHECK(node_) << "ERROR: Create v2x os interface node failed.";
  ACHECK(InitReaders()) << "ERROR: Initial readers failed.";
  ACHECK(InitWriters()) << "ERROR: Initial writers failed.";
  PrintModuleDetails();
  AINFO << "v2x os interface initial success";

  init_flag_ = true;
}

OsInterFace::~OsInterFace() {}

bool OsInterFace::InitReaders() {
  localization_reader_ =
      node_->CreateReader<LocalizationEstimate>(FLAGS_localization_topic);
  if (localization_reader_ == nullptr) {
    AERROR << "Create localization reader failed";
    return false;
  }
  perception_obstacle_reader_ =
      node_->CreateReader<PerceptionObstacles>(FLAGS_perception_obstacle_topic);
  if (perception_obstacle_reader_ == nullptr) {
    AERROR << "Create perception obstacles reader failed";
    return false;
  }
  return true;
}

bool OsInterFace::InitWriters() {
  v2x_obstacle_writer_ =
      node_->CreateWriter<PerceptionObstacles>(FLAGS_v2x_obstacle_topic);
  if (v2x_obstacle_writer_ == nullptr) {
    AERROR << "Create v2x obstacle writer failed";
    return false;
  }
  v2x_trafficlight_writer_ = node_->CreateWriter<IntersectionTrafficLightData>(
      FLAGS_v2x_trafficlight_topic);
  if (v2x_trafficlight_writer_ == nullptr) {
    AERROR << "Create v2x trafficlight writer failed";
    return false;
  }
  return true;
}

void OsInterFace::GetLocalizationFromOs(
    const std::shared_ptr<LocalizationEstimate> &msg) {
  ADEBUG << "get localization result from os";
  GetMsgFromOs(localization_reader_.get(), msg);
  if (FLAGS_debug_flag) {
    ADEBUG << "localization result: " << msg->DebugString();
  }
}

void OsInterFace::GetObstaclesFromOs(
    const std::shared_ptr<PerceptionObstacles> &msg) {
  ADEBUG << "get obstacles results from os";
  GetMsgFromOs(perception_obstacle_reader_.get(), msg);
  if (FLAGS_debug_flag) {
    ADEBUG << "perception result: " << msg->DebugString();
  }
}

void OsInterFace::SendV2xObstaclesToOs(
    const std::shared_ptr<PerceptionObstacles> &msg) {
  ADEBUG << "send v2x obstacles to os";
  SendMsgToOs(v2x_obstacle_writer_.get(), msg);
  if (FLAGS_debug_flag) {
    ADEBUG << "v2x obstacles result: " << msg->DebugString();
  }
}

void OsInterFace::SendV2xTrafficLightToOs(
    const std::shared_ptr<IntersectionTrafficLightData> &msg) {
  ADEBUG << "send v2x trafficlight to os";
  SendMsgToOs(v2x_trafficlight_writer_.get(), msg);
  if (FLAGS_debug_flag) {
    ADEBUG << "v2x trafficlight result: " << msg->DebugString();
  }
}

}  // namespace v2x
}  // namespace apollo
