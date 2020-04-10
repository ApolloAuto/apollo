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
#include "modules/dreamview/proto/hmi_status.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
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
  struct ADCCurrentInfo {
    std::pair<double, double> adc_cur_position_;
    std::pair<double, double> adc_cur_velocity_;
    std::pair<double, double> adc_cur_acc_;
    double adc_cur_heading_;
  };

  void OnChassis(const apollo::canbus::Chassis& chassis);
  void OnHMIStatus(apollo::dreamview::HMIStatus hmi_status);
  void OnLocalization(const apollo::localization::LocalizationEstimate& le);
  void OnPrediction(
      const apollo::prediction::PredictionObstacles& prediction_obstacles);
  void OnRoutingResponse(
      const apollo::routing::RoutingResponse& routing_response);
  void OnTafficLightDetection(
      const apollo::perception::TrafficLightDetection& traffic_light_detection);

  apollo::hdmap::LaneInfoConstPtr GetLane(
      const apollo::common::PointENU& position, int* routing_index);
  apollo::hdmap::LaneInfoConstPtr GetADCCurrentLane(int* routing_index);

  void GetADCCurrentInfo(ADCCurrentInfo* adc_curr_info);

  void GenerateObstacleTrajectoryPoint(
      const int obstacle_id,
      const ADCCurrentInfo& adc_curr_info,
      ObstacleFeature* obstacle_feature);

  void GenerateObstaclePrediction(
      const apollo::prediction::PredictionObstacle& prediction_obstacle,
      const ADCCurrentInfo& adc_curr_info,
      ObstacleFeature* obstacle_feature);

  void GenerateObstacleFeature(LearningDataFrame* learning_data_frame);

  void GenerateRoutingFeature(const int routing_index,
                              LearningDataFrame* learning_data_frame);

  void GenerateADCTrajectoryPoints(
      const std::list<apollo::localization::LocalizationEstimate>&
          localization_for_label,
      LearningDataFrame* learning_data_frame);

  void GenerateLearningDataFrame();

  void WriteOutLearningData(const LearningData& learning_data,
                            const int learning_data_file_index);

 private:
  std::string record_file_name_;
  std::unordered_map<std::string, std::string> map_m_;
  LearningData learning_data_;
  int learning_data_file_index_ = 0;
  std::list<apollo::localization::LocalizationEstimate>
      localization_for_label_;

  std::unordered_map<int, apollo::prediction::PredictionObstacle>
      prediction_obstacles_map_;
  std::unordered_map<int, std::list<PerceptionObstacleFeature>>
      obstacle_history_map_;
  ChassisFeature chassis_feature_;
  std::string map_name_;
  std::vector<OverlapFeature> overlaps_;
  std::vector<std::pair<std::string, double>> routing_lane_segment_;
  std::unordered_map<std::string, apollo::perception::TrafficLight::Color>
        traffic_lights_;
  int total_learning_data_frame_num_ = 0;
};

}  // namespace planning
}  // namespace apollo
