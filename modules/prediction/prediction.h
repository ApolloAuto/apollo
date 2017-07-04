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

#include "third_party/ros/include/ros/ros.h"

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class Prediction : public apollo::common::ApolloApp {
 public:
  virtual std::string Name() const override;
  virtual apollo::common::Status Init() override;
  virtual apollo::common::Status Start() override;
  virtual void Stop() override;
  ~Prediction(){};

 private:
  void OnPerception(
      const ::apollo::perception::PerceptionObstacles &perception_obstacles);
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTION_H_
