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

#ifndef MODULES_PREDICTION_PREDICTION_INTERFACE_H_
#define MODULES_PREDICTION_PREDICTION_INTERFACE_H_

#include "modules/common/apollo_app.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

/**
 * @class PredictionInterface
 *
 * @brief Interface of the prediction module
 */
class PredictionInterface : public apollo::common::ApolloApp {
 public:
  /**
   * @brief main logic of the prediction module, triggered upon receiving a new
   * frame of perception obstacle message.
   */
  virtual void RunOnce(
      const perception::PerceptionObstacles &perception_obstacles) = 0;
};

}  // namespace prediction
}  // namespace apollo

#endif /* MODULES_PREDICTION_PREDICTION_INTERFACE_H_ */
