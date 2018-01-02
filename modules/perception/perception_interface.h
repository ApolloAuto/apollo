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

#ifndef MODULES_PERCEPTION_PERCEPTION_INTERFACE_H_
#define MODULES_PERCEPTION_PERCEPTION_INTERFACE_H_

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/apollo_app.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "sensor_msgs/PointCloud2.h"

/**
 * @namespace apollo::perception
 * @brief apollo::perception
 */
namespace apollo {
namespace perception {

/**
 * @class PerceptionInterface
 *
 * @brief Interface of the perception module
 */
class PerceptionInterface : public apollo::common::ApolloApp {
 public:
  /**
   * @brief main logic of the perception module, triggered upon receiving a new
   * frame of PointCloud.
   */
  virtual void RunOnce(const sensor_msgs::PointCloud2& message) = 0;

  /**
   * @brief Fill the header and publish the perception message.
   */
  void Publish(perception::PerceptionObstacles* perception_obstacles) {
    using apollo::common::adapter::AdapterManager;
    AdapterManager::FillPerceptionObstaclesHeader(FLAGS_obstacle_module_name,
                                                  perception_obstacles);
    AdapterManager::PublishPerceptionObstacles(*perception_obstacles);
  }
};

}  // namespace perception
}  // namespace apollo

#endif /* MODULES_PERCEPTION_PERCEPTION_INTERFACE_H_ */
