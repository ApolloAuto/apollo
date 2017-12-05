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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_HDMAP_INPUT_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_HDMAP_INPUT_H_

#include <memory>
#include <mutex>
#include <vector>

#include "Eigen/Core"

#include "modules/common/macro.h"
#include "modules/map/hdmap/hdmap.h"

namespace apollo {
namespace perception {
namespace traffic_light {
// Singleton HDMapInput, interfaces are thread-safe.
class HDMapInput {
 public:
  bool Init();

  // @brief: get roi polygon
  //         all points are in the world frame
  bool GetSignals(const Eigen::Matrix4d &pointd,
                  std::vector<apollo::hdmap::Signal> *signals);

 private:
  std::mutex mutex_;  // multi-thread init safe.
  DECLARE_SINGLETON(HDMapInput);
};

typedef typename std::shared_ptr<HDMapInput> HDMapInputPtr;

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_HDMAP_INPUT_H_
