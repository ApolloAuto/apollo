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
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "modules/common/util/eigen_defs.h"

namespace apollo {
namespace perception {
namespace base {

struct ImpendingCollisionEdge {
  // edge id per frame
  int id = 0;

  // age of the tracked object
  double tracking_time = 0.0;

  // points of the edge
  apollo::common::EigenVector<Eigen::Vector3d> points;
};

// TODO(all): to remove
// typedef std::shared_ptr<ImpendingCollisionEdge> ImpendingCollisionEdgePtr;
// typedef std::shared_ptr<const ImpendingCollisionEdge>
//     ImpendingCollisionEdgeConstPtr;

// Sensor single frame objects.
struct ImpendingCollisionEdges {
  double timestamp = 0.0;

  // edges
  std::vector<std::shared_ptr<ImpendingCollisionEdge>>
      impending_collision_edges;

  // sensor to world position
  Eigen::Matrix4d sensor2world_pose = Eigen::Matrix4d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// TODO(all): to remove
// typedef std::shared_ptr<ImpendingCollisionEdges> ImpendingCollisionEdgesPtr;
// typedef std::shared_ptr<const ImpendingCollisionEdges>
//    ImpendingCollisionEdgesConstPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo
