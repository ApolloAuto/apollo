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
 * @file
 */

#pragma once

#include <memory>
#include <string>

#include "cyber/component/component.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class PredictionComponent
    : public cyber::Component<perception::PerceptionObstacles> {
 public:
  /**
   * @brief Destructor
   */
  ~PredictionComponent();

  /**
   * @brief Get name of the node
   * @return Name of the node
   */
  std::string Name() const;

  /**
   * @brief Initialize the node
   * @return If initialized
   */
  bool Init() override;

  /**
   * @brief Data callback upon receiving a perception obstacle message.
   * @param Perception obstacle message.
   */
  bool Proc(
      const std::shared_ptr<perception::PerceptionObstacles> &) override;

  /**
   * @brief Load and process feature proto file.
   * @param a bin file including a sequence of feature proto.
   */
  void OfflineProcessFeatureProtoFile(const std::string& features_proto_file);

 private:
  void OnLocalization(const localization::LocalizationEstimate &localization);

  void OnPlanning(const planning::ADCTrajectory &adc_trajectory);

  void OnPerception(
      const perception::PerceptionObstacles &perception_obstacles);

  void ProcessOfflineData(const std::string &filename);

 private:
  double component_start_time_ = 0.0;

  double frame_start_time_ = 0.0;

  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>>
      planning_reader_;

  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_;

  std::shared_ptr<cyber::Writer<PredictionObstacles>>
      prediction_writer_;
};

CYBER_REGISTER_COMPONENT(PredictionComponent)

}  // namespace prediction
}  // namespace apollo
