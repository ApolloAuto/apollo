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

#include "modules/v2x/fusion/apps/v2x_fusion_component.h"

namespace apollo {
namespace v2x {
namespace ft {

V2XFusionComponent::~V2XFusionComponent() {}

std::string V2XFusionComponent::Name() const { return FLAGS_v2x_module_name; }

bool V2XFusionComponent::Init() {
  v2x_obstacles_reader_ =
      node_->CreateReader<V2XObstacles>(FLAGS_v2x_obstacle_topic, nullptr);
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic, nullptr);
  perception_fusion_obstacles_writer_ =
      node_->CreateWriter<PerceptionObstacles>(FLAGS_perception_obstacle_topic);
  return true;
}

bool V2XFusionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  if (FLAGS_use_v2x) {
    return V2XMessageFusionProcess(perception_obstacles);
  }
  perception_fusion_obstacles_writer_->Write(*perception_obstacles);
  return true;
}

bool V2XFusionComponent::V2XMessageFusionProcess(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  localization_reader_->Observe();
  auto localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "V2X: cannot receive any localization message.";
    return false;
  }
  base::Object hv_obj;
  CarstatusPb2Object(*localization_msg, &hv_obj, "VEHICLE");
  v2x_obstacles_reader_->Observe();
  auto v2x_obstacles_msg = v2x_obstacles_reader_->GetLatestObserved();
  if (v2x_obstacles_msg == nullptr) {
    AERROR << "V2X: cannot receive any v2x obstacles message.";
    perception_fusion_obstacles_writer_->Write(*perception_obstacles);
  } else {
    header_.CopyFrom(perception_obstacles->header());
    std::vector<Object> fused_objects;
    std::vector<Object> v2x_fused_objects;
    std::vector<std::vector<Object>> fusion_result;
    std::vector<Object> v2x_objects;
    V2xPbs2Objects(*v2x_obstacles_msg, &v2x_objects, "V2X");
    std::vector<Object> perception_objects;
    Pbs2Objects(*perception_obstacles, &perception_objects, "VEHICLE");
    perception_objects.push_back(hv_obj);
    fusion_.CombineNewResource(perception_objects, &fused_objects,
                               &fusion_result);
    fusion_.CombineNewResource(v2x_objects, &fused_objects, &fusion_result);
    fusion_.GetV2xFusionObjects(fusion_result, &v2x_fused_objects);
    auto output_msg = std::make_shared<PerceptionObstacles>();
    SerializeMsg(v2x_fused_objects, output_msg);
    perception_fusion_obstacles_writer_->Write(*output_msg);
  }
  return true;
}

void V2XFusionComponent::SerializeMsg(
    const std::vector<base::Object>& objects,
    std::shared_ptr<PerceptionObstacles> output_msg) {
  static int seq_num = 0;
  Objects2Pbs(objects, output_msg);
  apollo::common::Header* header = output_msg->mutable_header();
  header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
  header->set_module_name("v2x_fusion");
  header->set_sequence_num(seq_num++);
  header->set_lidar_timestamp(header_.lidar_timestamp());
  header->set_camera_timestamp(header_.camera_timestamp());
  header->set_radar_timestamp(header_.radar_timestamp());
  output_msg->set_error_code(apollo::common::ErrorCode::PERCEPTION_ERROR_NONE);
}

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
