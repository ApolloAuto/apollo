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
#ifndef PERCEPTION_MAP_HDMAP_HDMAP_INPUT_H_
#define PERCEPTION_MAP_HDMAP_HDMAP_INPUT_H_
#include <gflags/gflags.h>
// #include <hdmap.h>
#include <string>
#include <vector>
#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/base/point_cloud_types.h"
#include "modules/perception/lib/singleton/singleton.h"
#include "modules/perception/lib/thread/mutex.h"
namespace apollo {
namespace perception {
namespace map {
class HDMapInput {
 public:
  // thread safe
  bool Init();
  bool Reset();
  bool GetRoiHDMapStruct(const base::PointD& pointd, const double distance,
                         base::HdmapStructPtr hdmap_struct_prt);
  bool GetNearestLaneDirection(const base::PointD& pointd,
                               Eigen::Vector3d* lane_direction);
  //   bool GetSignals(const Eigen::Vector3d& pointd,
  //       double forward_distance,
  //       std::vector<adu::common::hdmap::Signal>* signals);

 private:
  HDMapInput() = default;
  HDMapInput(const HDMapInput&) = delete;
  HDMapInput& operator=(const HDMapInput&) = delete;
  bool InitHDMap();
  bool InitInternal();
  //   void MergeBoundaryJunction(
  //       const std::vector<adu::hdmap::RoiRoadBoundaryPtr>& boundary,
  //       const std::vector<adu::hdmap::JunctionInfoConstPtr>& junctions,
  //       std::vector<base::RoadBoundary>* road_boundaries_ptr,
  //       std::vector<base::PolygonDType>* road_polygons_ptr,
  //       std::vector<base::PolygonDType>* junction_polygons_ptr);
  bool GetRoadBoundaryFilteredByJunctions(
      const std::vector<base::RoadBoundary>& road_boundaries,
      const std::vector<base::PolygonDType>& junctions,
      std::vector<base::RoadBoundary>* flt_road_boundaries_ptr);
  void DownsamplePoints(const base::PointDCloudPtr& raw_cloud_ptr,
                        base::PolygonDType* polygon_ptr,
                        int min_points_num_for_sample = 15) const;
  void SplitBoundary(const base::PolygonDType& boundary_line,
                     const std::vector<base::PolygonDType>& junctions,
                     std::vector<base::PolygonDType>* boundary_line_vec_ptr);
  //   bool GetSignalsFromHDMap(const Eigen::Vector3d& pointd,
  //     double forward_distance,
  //     std::vector<adu::common::hdmap::Signal>* signals);
  bool inited_ = false;
  lib::Mutex mutex_;
  //   std::unique_ptr<adu::hdmap::HDMap> hdmap_;
  //   int hdmap_sample_step_ = 5;
  //   std::string hdmap_file_;
  friend class lib::Singleton<HDMapInput>;
};
}  // namespace map
}  // namespace perception
}  // namespace apollo
#endif  // PERCEPTION_MAP_HDMAP_HDMAP_INPUT_H_
