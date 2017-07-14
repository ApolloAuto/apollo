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

#ifndef MODULES_PREDICTION_PREDICTION_H_
#define MODULES_PREDICTION_PREDICTION_H_

#include "ros/include/ros/ros.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/common/apollo_app.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class Prediction : public apollo::common::ApolloApp {
 public:
  /**
   * @brief Destructor
   */
  virtual ~Prediction() = default;

  /**
   * @brief Get name of the node
   * @return Name of the node
   */
  virtual std::string Name() const override;

  /**
   * @brief Initialize the node
   * @return Status of the initialization
   */
  virtual ::apollo::common::Status Init() override;

  /**
   * @brief Start the node
   * @return Status of the starting process
   */
  virtual ::apollo::common::Status Start() override;

  /**
   * @brief Stop the node
   */
  virtual void Stop() override;

 private:
  ::apollo::common::Status OnError(const std::string& error_msg);

  void OnPerception(
      const perception::PerceptionObstacles &perception_obstacles);

 private:
  PredictionConf conf_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTION_H_
