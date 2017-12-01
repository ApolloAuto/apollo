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

#include "modules/perception/obstacle/onboard/hdmap_input.h"

#include <stdlib.h>
#include <algorithm>
#include <vector>

#include "Eigen/Core"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/common/define.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

using apollo::hdmap::LineSegment;
using apollo::hdmap::BoundaryEdge;
using apollo::hdmap::RoadROIBoundaryPtr;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::JunctionBoundaryPtr;
using apollo::hdmap::BoundaryEdge_Type_LEFT_BOUNDARY;
using apollo::hdmap::BoundaryEdge_Type_RIGHT_BOUNDARY;
using apollo::hdmap::HDMapUtil;
using apollo::common::math::Vec2d;
using apollo::common::math::Polygon2d;
using pcl_util::PointD;
using pcl_util::PointDCloud;
using pcl_util::PointDCloudPtr;
using std::string;
using std::vector;

// HDMapInput
HDMapInput::HDMapInput() {}

bool HDMapInput::Init() {
  return HDMapUtil::ReloadMaps();
}

bool HDMapInput::GetROI(const PointD& pointd, const double& map_radius,
                        HdmapStructPtr* mapptr) {
  auto* hdmap = HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    return false;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  if (mapptr != NULL && *mapptr == nullptr) {
    (*mapptr).reset(new HdmapStruct);
  }
  common::PointENU point;
  point.set_x(pointd.x);
  point.set_y(pointd.y);
  point.set_z(pointd.z);
  std::vector<RoadROIBoundaryPtr> boundary_vec;
  std::vector<JunctionBoundaryPtr> junctions_vec;

  int status = hdmap->GetRoadBoundaries(point, map_radius, &boundary_vec,
                                        &junctions_vec);
  if (status != SUCC) {
    AERROR << "Failed to get road boundaries for point " << point.DebugString();
    return false;
  }
  ADEBUG << "Get road boundaries : num_boundary = " << boundary_vec.size()
         << " num_junction = " << junctions_vec.size();

  if (MergeBoundaryJunction(boundary_vec, junctions_vec, mapptr) != SUCC) {
    AERROR << "merge boundary and junction to hdmap struct failed.";
    return false;
  }
  return true;
}

bool HDMapInput::GetNearestLaneDirection(const pcl_util::PointD& pointd,
                                         Eigen::Vector3d* lane_direction) {
  auto* hdmap = HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    return false;
  }
  common::PointENU point;
  point.set_x(pointd.x);
  point.set_y(pointd.y);
  point.set_z(pointd.z);
  hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;

  int status =
      hdmap->GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l);
  if (status != SUCC) {
    AERROR << "Failed to get nearest lane for point " << point.DebugString();
    return false;
  }
  double lane_heading = nearest_lane->Heading(nearest_s);
  (*lane_direction) = Eigen::Vector3d(cos(lane_heading), sin(lane_heading), 0);
  return true;
}

int HDMapInput::MergeBoundaryJunction(
    const std::vector<RoadROIBoundaryPtr>& boundaries,
    const std::vector<JunctionBoundaryPtr>& junctions, HdmapStructPtr* mapptr) {
  if (*mapptr == nullptr) {
    AERROR << "the HdmapStructPtr mapptr is null";
    return FAIL;
  }
  (*mapptr)->road_boundary.resize(boundaries.size());
  (*mapptr)->junction.resize(junctions.size());

  for (size_t i = 0; i < boundaries.size(); i++) {
    for (const auto& road_boundary : boundaries[i]->road_boundaries()) {
      auto& mapped_boundary = (*mapptr)->road_boundary[i];
      for (const BoundaryEdge& edge : road_boundary.outer_polygon().edge()) {
        PolygonDType* edge_side = nullptr;
        if (edge.type() == BoundaryEdge::LEFT_BOUNDARY) {
          edge_side = &mapped_boundary.left_boundary;
        } else if (edge.type() == BoundaryEdge::RIGHT_BOUNDARY) {
          edge_side = &mapped_boundary.right_boundary;
        } else {
          continue;
        }
        for (const auto& segment : edge.curve().segment()) {
          if (segment.has_line_segment()) {
            DownSampleBoundary(segment.line_segment(), edge_side);
          }
        }
      }
    }
  }

  for (size_t i = 0; i < junctions.size(); i++) {
    const Polygon2d& polygon = junctions[i]->junction_info->polygon();
    const vector<Vec2d>& points = polygon.points();
    auto& junction = (*mapptr)->junction[i];
    junction.reserve(points.size());
    for (const auto& point : points) {
      PointD pointd;
      pointd.x = point.x();
      pointd.y = point.y();
      pointd.z = 0.0;
      junction.push_back(pointd);
    }
  }

  return SUCC;
}

void HDMapInput::DownSampleBoundary(const hdmap::LineSegment& line,
                                    PolygonDType* out_boundary_line) const {
  PointDCloudPtr raw_cloud(new PointDCloud);
  for (int i = 0; i < line.point_size(); ++i) {
    if (i % FLAGS_map_sample_step == 0) {
      PointD pointd;
      pointd.x = line.point(i).x();
      pointd.y = line.point(i).y();
      pointd.z = line.point(i).z();
      raw_cloud->push_back(pointd);
    }
  }

  const double kCumulateThetaError = 5.0;
  size_t spt = 0;
  double acos_theta = 0.0;
  size_t raw_cloud_size = raw_cloud->points.size();
  if (raw_cloud_size <= 3) {
    for (const auto& point : raw_cloud->points) {
      out_boundary_line->push_back(point);
    }
    ADEBUG << "Points num < 3, so no need to downsample.";
    return;
  }
  out_boundary_line->points.reserve(out_boundary_line->points.size() +
                                    raw_cloud_size);
  // the first point
  out_boundary_line->push_back(raw_cloud->points[0]);
  for (size_t i = 2; i < raw_cloud_size; ++i) {
    const PointD& point_0 = raw_cloud->points[spt];
    const PointD& point_1 = raw_cloud->points[i - 1];
    const PointD& point_2 = raw_cloud->points[i];
    Eigen::Vector2d v1(point_1.x - point_0.x, point_1.y - point_0.y);
    Eigen::Vector2d v2(point_2.x - point_1.x, point_2.y - point_1.y);
    double vector_dist =
        sqrt(v1.cwiseProduct(v1).sum()) * sqrt(v2.cwiseProduct(v2).sum());
    // judge duplicate points
    if (vector_dist < DBL_EPSILON) {
      continue;
    }
    double cos_theta = (v1.cwiseProduct(v2)).sum() / vector_dist;
    if (cos_theta > 1.0) {
      cos_theta = 1.0;
    } else if (cos_theta < -1.0) {
      cos_theta = -1.0;
    }
    double angle = (acos(cos_theta) * kRadianToDegree);
    acos_theta += angle;
    if ((acos_theta - kCumulateThetaError) > DBL_EPSILON) {
      out_boundary_line->push_back(point_1);
      spt = i - 1;
      acos_theta = 0.0;
    }
  }
  // the last point
  out_boundary_line->push_back(raw_cloud->points[raw_cloud_size - 1]);
}

}  // namespace perception
}  // namespace apollo
