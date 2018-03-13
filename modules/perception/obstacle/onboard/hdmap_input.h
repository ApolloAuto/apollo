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

#ifndef MODULES_PERCEPTION_ONBOARD_HDMAP_INPUT_H_
#define MODULES_PERCEPTION_ONBOARD_HDMAP_INPUT_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/common/macro.h"
#include "modules/common/status/status.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

// Singleton HDMapInput, interfaces are thread-safe.
class HDMapInput {
 public:
  bool Init();

  // @brief: get roi polygon
  //         all points are in the world frame
  bool GetROI(const pcl_util::PointD& pointd, const double& map_radius,
              HdmapStructPtr* mapptr);

  // @brief: get nearest lane direction
  bool GetNearestLaneDirection(const pcl_util::PointD& pointd,
                               Eigen::Vector3d* lane_direction);

 private:
  void DownSampleBoundary(const hdmap::LineSegment& line,
                          PolygonDType* out_boundary_line) const;

  apollo::common::Status MergeBoundaryJunction(
      const std::vector<hdmap::RoadROIBoundaryPtr>& boundaries,
      const std::vector<hdmap::JunctionBoundaryPtr>& junctions,
      HdmapStructPtr* mapptr);

  std::mutex mutex_;  // multi-thread init safe.

  FRIEND_TEST(HDMapInputTest, test_Init);
  FRIEND_TEST(HDMapInputTest, test_GetROI);

  DECLARE_SINGLETON(HDMapInput);
};

typedef typename std::shared_ptr<HDMapInput> HDMapInputPtr;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_HDMAP_INPUT_H_
