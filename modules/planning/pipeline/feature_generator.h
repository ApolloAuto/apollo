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
#pragma once

#include <list>
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>

#include "cyber/common/file.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace planning {

class FeatureGenerator {
 public:
  void Init();
  void Close();

  void ProcessOfflineData(const std::string& record_filename);

 private:
  void OnChassis(const apollo::canbus::Chassis& chassis);
  void OnLocalization(const apollo::localization::LocalizationEstimate& le);
  void OnPrediction(
      const apollo::prediction::PredictionObstacles& prediction_obstacles);
  void OnRoutingResponse(
      const apollo::routing::RoutingResponse& routing_response);
  void OnTafficLightDetection(
      const apollo::perception::TrafficLightDetection& traffic_light_detection);

  void GenerateObstacleData(LearningDataFrame* learning_data_frame);

  void GenerateADCTrajectoryPoints(
      const std::list<apollo::localization::LocalizationEstimate>&
          localization_for_label,
      LearningDataFrame* learning_data_frame);

  void GenerateLearningDataFrame();

  void WriteOutLearningData(const LearningData& learning_data,
                            const std::string& file_name);

 private:
  LearningData learning_data_;
  int learning_data_file_index_ = 0;
  std::list<apollo::localization::LocalizationEstimate>
      localization_for_label_;

  std::unordered_map<int, apollo::prediction::PredictionObstacle>
      prediction_obstacles_map_;
  std::unordered_map<int, std::list<ObstacleTrajectoryPoint>>
      obstacle_history_map_;

  ChassisFeature chassis_feature_;
  std::vector<std::string> routing_lane_ids_;
  std::unordered_map<std::string, apollo::perception::TrafficLight::Color>
        traffic_lights_;
  int total_learning_data_frame_num_ = 0;
};

}  // namespace planning
}  // namespace apollo
