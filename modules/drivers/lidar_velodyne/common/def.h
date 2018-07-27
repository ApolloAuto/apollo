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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_COMMON_DEF_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_COMMON_DEF_H_

#include <set>
#include "modules/drivers/lidar_velodyne/proto/velodyne_conf.pb.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

static const std::set<VelodyneModel> v64_models = {
      V64E_S2,
      V64E_S3S,
      V64E_S3D_STRONGEST,
      V64E_S3D_LAST,
      V64E_S3D_DUAL
};
static const std::set<VelodyneModel> v16_models = {VLP16};
static const char* valid_models =
    "V64E_S2|V64E_S3S|V64E_S3D_STRONGEST|V64E_S3D_LAST|V64E_S3D_DUAL|VLP16";
}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_COMMON_DEF_H_
