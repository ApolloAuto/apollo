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

#include <mutex>
#include "gflags/gflags.h"

#include "modules/common/macro.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/perception/lib/base/singleton.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"

namespace apollo {
namespace perception {

// Singleton HDMapInput, interfaces are thread-safe.
class HDMapInput {
 public:
  // thread safe
  bool Init();

  // @brief: get roi polygon
  //         all points are in the world frame
  bool GetROI(const pcl_util::PointD& pointd, HdmapStructPtr mapptr);

 private:
  HDMapInput() = default;

  /*
  void DownSampleBoundary(const apollo::hdmap::LineSegment& line,
                          PolygonDType* out_boundary_line) const;

  int MergeBoundaryJunction(
      std::vector<apollo::hdmap::RoadBoundaryPtr>& boundaries,
      std::vector<apollo::hdmap::JunctionBoundaryPtr>& junctions,
      HdmapStructPtr mapptr);
  */
  friend class Singleton<HDMapInput>;

  bool inited_ = false;
  std::mutex mutex_;  // multi-thread init safe.
  std::unique_ptr<apollo::hdmap::HDMap> hdmap_;

  DISALLOW_COPY_AND_ASSIGN(HDMapInput);
};

}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_ONBOARD_HDMAP_INPUT_H
