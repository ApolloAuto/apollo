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

#include "cyber/time/time.h"

#include "cyber/component/component.h"
#include "modules/prediction/common/message_process.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/submodules/submodule_output.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"

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
  bool Proc(const std::shared_ptr<perception::PerceptionObstacles>&) override;

  /**
   * @brief Load and process feature proto file.
   * @param a bin file including a sequence of feature proto.
   */
  void OfflineProcessFeatureProtoFile(const std::string& features_proto_file);

 private:
  bool ContainerSubmoduleProcess(
      const std::shared_ptr<perception::PerceptionObstacles>&);

  bool PredictionEndToEndProc(
      const std::shared_ptr<perception::PerceptionObstacles>&);

  double component_start_time_ = 0.0;

  double frame_start_time_ = 0.0;

  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>> planning_reader_;

  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_;

  std::shared_ptr<cyber::Reader<storytelling::Stories>> storytelling_reader_;

  std::shared_ptr<cyber::Writer<PredictionObstacles>> prediction_writer_;

  std::shared_ptr<cyber::Writer<SubmoduleOutput>> container_writer_;

  std::shared_ptr<cyber::Writer<ADCTrajectoryContainer>> adc_container_writer_;

  std::shared_ptr<cyber::Writer<perception::PerceptionObstacles>>
      perception_obstacles_writer_;

  std::shared_ptr<ContainerManager> container_manager_;

  std::unique_ptr<EvaluatorManager> evaluator_manager_;

  std::unique_ptr<PredictorManager> predictor_manager_;

  std::unique_ptr<ScenarioManager> scenario_manager_;
};

CYBER_REGISTER_COMPONENT(PredictionComponent)

}  // namespace prediction
}  // namespace apollo
