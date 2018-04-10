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

#ifndef MODULES_DRIVERS_PANDORA_PANDORA_POINTCLOUD_CONST_VARIABLES_H_
#define MODULES_DRIVERS_PANDORA_PANDORA_POINTCLOUD_CONST_VARIABLES_H_

#include <iostream>

namespace apollo {
namespace drivers {
namespace pandora {

// default topics
const std::string TOPIC_PREFIX = "/";
const std::string TOPIC_POINTCLOUD = TOPIC_PREFIX + "pandar_points";
const std::string TOPIC_COMPENSATED_POINTCLOUD =
    TOPIC_PREFIX + "compensator/pandar_points";
}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_CONST_VARIABLES_H_
