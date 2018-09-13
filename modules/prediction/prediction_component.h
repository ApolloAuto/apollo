/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PREDICTION_PREDICTION_COMPONENT_H_
#define MODULES_PREDICTION_PREDICTION_COMPONENT_H_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "cybertron/component/component.h"
#include "modules/common/status/status.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class PredictionComponent :
    public cybertron::Component<perception::PerceptionObstacles,
                                localization::LocalizationEstimate,
                                planning::ADCTrajectory> {
 public:
  /**
   * @brief Destructor
   */
  ~PredictionComponent() = default;

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
   * @param perception_obstacles received message.
   */
  bool Proc(
      const std::shared_ptr<perception::PerceptionObstacles>&,
      const std::shared_ptr<localization::LocalizationEstimate>&,
      const std::shared_ptr<planning::ADCTrajectory>&) override;

  /**
   * @brief Start the node
   * @return Status of the starting process
   */
  common::Status Start();

  /**
   * @brief Stop the node
   */
  void Stop();

 private:
  common::Status OnError(const std::string &error_msg);

  void OnLocalization(const localization::LocalizationEstimate &localization);

  void OnPlanning(const planning::ADCTrajectory &adc_trajectory);

  /**
   * @brief process rosbag in offline mode, mainly for extracting prediction
   * features.
   */
  void ProcessOfflineData(const std::string &filename);

 private:
  double start_time_ = 0.0;

  PredictionConf prediction_conf_;

  common::adapter::AdapterManagerConfig adapter_conf_;
  std::shared_ptr<Reader<localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<apollo::cybertron::Reader<planning::ADCTrajectory>>
      planning_reader_;

  std::shared_ptr<apollo::cybertron::Writer<PredictionObstacles>>
      prediction_writer_;

  std::mutex mutex_;
};

CYBERTRON_REGISTER_COMPONENT(PredictionComponent)

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTION_COMPONENT_H_
