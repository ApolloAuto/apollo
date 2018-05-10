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
#ifndef MODULES_PERCEPTION_ONBOARD_TRANSFORM_INPUT_H_
#define MODULES_PERCEPTION_ONBOARD_TRANSFORM_INPUT_H_

#include "Eigen/Core"

namespace apollo {
namespace perception {

bool GetVelodyneTrans(const double query_time, Eigen::Matrix4d* trans);

bool GetRadarTrans(const double query_time, Eigen::Matrix4d* trans);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_TRANSFORM_INPUT_H_
