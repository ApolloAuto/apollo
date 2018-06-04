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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_ULTRASONIC_OBSTACLE_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_ULTRASONIC_OBSTACLE_SUBNODE_H_

#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <set>
#include "Eigen/Dense"
#include "Eigen/Core"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/perception/proto/perception_ultrasonic.pb.h"
#include "modules/perception/onboard/subnode.h"

namespace apollo {
namespace perception {

class UltrasonicObstacleSubnode : public Subnode {
 public:
  UltrasonicObstacleSubnode() : seq_num_(0) {}

  ~UltrasonicObstacleSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  void OnUltrasonic(const apollo::canbus::Chassis& message);

  bool InitInternal() override;

  void RegistAllAlgorithm();

  bool init_algorithm_plugin();

  bool set_ultrasonic_type(const std::string& type);

 private:
  uint32_t seq_num_;

  std::string device_id_;

  DISALLOW_COPY_AND_ASSIGN(UltrasonicObstacleSubnode);
};

REGISTER_SUBNODE(UltrasonicObstacleSubnode);
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_ULTRASONIC_OBSTACLE_SUBNODE_H_
