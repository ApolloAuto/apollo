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

#include "modules/perception/map/hdmap/hdmap_input.h"

#include <algorithm>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace map {

using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::LineBoundary;
using apollo::hdmap::RoadRoiPtr;
using apollo::hdmap::SignalInfoConstPtr;

using base::PointD;
using base::PointDCloud;
using base::PointDCloudPtr;
using base::RoadBoundary;
// HDMapInput

HDMapInput::HDMapInput() {}

bool HDMapInput::Init() {
  lib::MutexLock lock(&mutex_);
  return InitInternal();
}

bool HDMapInput::InitInternal() {
  if (inited_) {
    return true;
  }
  if (!InitHDMap()) {
    return false;
  }
  inited_ = true;
  return true;
}

bool HDMapInput::Reset() {
  lib::MutexLock lock(&mutex_);
  inited_ = false;
  return InitInternal();
}

bool HDMapInput::InitHDMap() {
  hdmap_.reset(new apollo::hdmap::HDMap());
  const std::string model_name = "HDMapInput";
  const lib::ModelConfig* model_config = nullptr;
  if (!lib::ConfigManager::Instance()->GetModelConfig(model_name,
                                                      &model_config)) {
    AERROR << "Failed to find model: " << model_name;
    return false;
  }
  if (!model_config->get_value("hdmap_sample_step", &hdmap_sample_step_)) {
    AERROR << "hdmap_sample_step not found.";
    hdmap_sample_step_ = 5;
  }

  // TO DO: Decide which map to use
  // Option 1: Use global hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  // hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();

  // Option2: Load own map with different hdmap_sample_step_
  // Load hdmap path from global_flagfile.txt
  hdmap_file_ = absl::StrCat(FLAGS_map_dir, "/base_map.bin");
  AINFO << "hdmap_file_: " << hdmap_file_;
  if (!apollo::cyber::common::PathExists(hdmap_file_)) {
    AERROR << "Failed to find hadmap file: " << hdmap_file_;
    return false;
  }
  if (hdmap_->LoadMapFromFile(hdmap_file_) != 0) {
    AERROR << "Failed to load hadmap file: " << hdmap_file_;
    return false;
  }

  AINFO << "Load hdmap file: " << hdmap_file_;
  return true;
}

bool HDMapInput::GetRoiHDMapStruct(
    const base::PointD& pointd, const double distance,
    std::shared_ptr<base::HdmapStruct> hdmap_struct_ptr) {
  lib::MutexLock lock(&mutex_);
  if (hdmap_.get() == nullptr) {
    AERROR << "hdmap is not available";
    return false;
  }
  // Get original road boundary and junction
  std::vector<RoadRoiPtr> road_boundary_vec;
  std::vector<JunctionInfoConstPtr> junctions_vec;
  apollo::common::PointENU point;
  point.set_x(pointd.x);
  point.set_y(pointd.y);
  point.set_z(pointd.z);
  if (hdmap_->GetRoadBoundaries(point, distance, &road_boundary_vec,
                                &junctions_vec) != 0) {
    AERROR << "Failed to get road boundary, point: " << point.DebugString();
    return false;
  }
  if (hdmap_struct_ptr == nullptr) {
    return false;
  }
  hdmap_struct_ptr->hole_polygons.clear();
  hdmap_struct_ptr->junction_polygons.clear();
  hdmap_struct_ptr->road_boundary.clear();
  hdmap_struct_ptr->road_polygons.clear();

  // Merge boundary and junction
  std::vector<RoadBoundary> road_boundaries;
  MergeBoundaryJunction(road_boundary_vec, junctions_vec, &road_boundaries,
                        &(hdmap_struct_ptr->road_polygons),
                        &(hdmap_struct_ptr->junction_polygons));
  // Filter road boundary by junction
  GetRoadBoundaryFilteredByJunctions(road_boundaries,
                                     hdmap_struct_ptr->junction_polygons,
                                     &(hdmap_struct_ptr->road_boundary));
  return true;
}

void HDMapInput::MergeBoundaryJunction(
    const std::vector<apollo::hdmap::RoadRoiPtr>& boundary,
    const std::vector<apollo::hdmap::JunctionInfoConstPtr>& junctions,
    std::vector<base::RoadBoundary>* road_boundaries_ptr,
    std::vector<base::PolygonDType>* road_polygons_ptr,
    std::vector<base::PolygonDType>* junction_polygons_ptr) {
  const int boundary_size = static_cast<int>(boundary.size());
  const int junctions_size = static_cast<int>(junctions.size());
  const int polygon_size = boundary_size;
  road_boundaries_ptr->clear();
  road_polygons_ptr->clear();
  junction_polygons_ptr->clear();
  road_polygons_ptr->resize(polygon_size);
  junction_polygons_ptr->resize(junctions_size);
  road_boundaries_ptr->resize(polygon_size);
  int polygons_index = 0;
  // Merge boundary
  PointDCloudPtr temp_cloud = base::PointDCloudPool::Instance().Get();
  for (int step = 0, i = 0; i < polygon_size; ++i) {
    temp_cloud->clear();
    const LineBoundary& left_boundary = boundary[i]->left_boundary;
    const std::vector<apollo::common::PointENU>& left_line_points =
        left_boundary.line_points;
    ADEBUG << "Input left road_boundary size = " << left_line_points.size();
    step = left_line_points.size() > 2 ? hdmap_sample_step_ : 1;
    for (unsigned int idx = 0; idx < left_line_points.size(); idx += step) {
      PointD pointd;
      pointd.x = left_line_points.at(idx).x();
      pointd.y = left_line_points.at(idx).y();
      pointd.z = left_line_points.at(idx).z();
      temp_cloud->push_back(pointd);
    }
    DownsamplePoints(temp_cloud,
                     &(road_boundaries_ptr->at(polygons_index).left_boundary));
    for (unsigned int index = 0;
         index < road_boundaries_ptr->at(polygons_index).left_boundary.size();
         ++index) {
      road_polygons_ptr->at(polygons_index)
          .push_back(
              road_boundaries_ptr->at(polygons_index).left_boundary[index]);
    }
    ADEBUG << "Left road_boundary downsample size = "
           << road_boundaries_ptr->at(polygons_index).left_boundary.size();
    temp_cloud->clear();
    const LineBoundary& right_boundary = boundary[i]->right_boundary;
    const std::vector<apollo::common::PointENU>& right_line_points =
        right_boundary.line_points;
    ADEBUG << "Input right road_boundary size = " << right_line_points.size();
    step = right_line_points.size() > 2 ? hdmap_sample_step_ : 1;
    for (unsigned int idx = 0; idx < right_line_points.size(); idx += step) {
      PointD pointd;
      pointd.x = right_line_points.at(idx).x();
      pointd.y = right_line_points.at(idx).y();
      pointd.z = right_line_points.at(idx).z();
      temp_cloud->push_back(pointd);
    }
    DownsamplePoints(temp_cloud,
                     &(road_boundaries_ptr->at(polygons_index).right_boundary));
    for (unsigned int index = 0;
         index < road_boundaries_ptr->at(polygons_index).right_boundary.size();
         ++index) {
      road_polygons_ptr->at(polygons_index)
          .push_back(road_boundaries_ptr->at(polygons_index)
                         .right_boundary[road_boundaries_ptr->at(polygons_index)
                                             .right_boundary.size() -
                                         1 - index]);
    }
    ADEBUG << "Right road_boundary downsample size = "
           << road_boundaries_ptr->at(polygons_index).right_boundary.size();
    ++polygons_index;
  }

  // Merge junctions
  for (int idx = 0; idx < junctions_size; ++idx) {
    const Polygon2d& polygon = junctions[idx]->polygon();
    const std::vector<Vec2d>& points = polygon.points();
    for (size_t idj = 0; idj < points.size(); ++idj) {
      PointD pointd;
      pointd.x = points[idj].x();
      pointd.y = points[idj].y();
      pointd.z = 0.0;
      junction_polygons_ptr->at(idx).push_back(pointd);
    }
  }
}

bool HDMapInput::GetRoadBoundaryFilteredByJunctions(
    const std::vector<base::RoadBoundary>& road_boundaries,
    const std::vector<base::PointCloud<PointD>>& junctions,
    std::vector<base::RoadBoundary>* flt_road_boundaries_ptr) {
  for (size_t n_rd = 0; n_rd < road_boundaries.size(); ++n_rd) {
    const base::RoadBoundary& temp_road_boundary = road_boundaries[n_rd];
    std::vector<base::PolygonDType> temp_left_boundary_vec;
    std::vector<base::PolygonDType> temp_right_boundary_vec;
    // Filter left boundary points
    this->SplitBoundary(temp_road_boundary.left_boundary, junctions,
                        &temp_left_boundary_vec);
    // Filter right boundary points
    this->SplitBoundary(temp_road_boundary.right_boundary, junctions,
                        &temp_right_boundary_vec);
    auto n_temp_road_boundary =
        std::max(temp_left_boundary_vec.size(), temp_right_boundary_vec.size());
    for (size_t i = 0; i < n_temp_road_boundary; ++i) {
      base::RoadBoundary temp_road_boundary;
      if (i < temp_left_boundary_vec.size()) {
        temp_road_boundary.left_boundary = temp_left_boundary_vec[i];
      }
      if (i < temp_right_boundary_vec.size()) {
        temp_road_boundary.right_boundary = temp_right_boundary_vec[i];
      }
      flt_road_boundaries_ptr->push_back(temp_road_boundary);
    }
  }

  return true;
}

void HDMapInput::DownsamplePoints(const base::PointDCloudPtr& raw_cloud_ptr,
                                  base::PointCloud<base::PointD>* polygon_ptr,
                                  size_t min_points_num_for_sample) const {
  const PointDCloud& raw_cloud = *raw_cloud_ptr;
  unsigned int spt = 0;
  double acos_theta = 0.0;
  const double radian_to_degree = 57.29577951308232;
  const size_t raw_cloud_size = raw_cloud.size();
  if (raw_cloud_size <= min_points_num_for_sample) {
    for (size_t i = 0; i < raw_cloud_size; ++i) {
      polygon_ptr->push_back(raw_cloud[i]);
    }
    return;
  }
  // The first point
  polygon_ptr->push_back(raw_cloud[0]);
  for (size_t idx = 2; idx < raw_cloud_size; ++idx) {
    const PointD& point_0 = raw_cloud[spt];
    const PointD& point_1 = raw_cloud[idx - 1];
    const PointD& point_2 = raw_cloud[idx];
    Eigen::Vector2d v1(point_1.x - point_0.x, point_1.y - point_0.y);
    Eigen::Vector2d v2(point_2.x - point_1.x, point_2.y - point_1.y);
    double vector_dist =
        sqrt(v1.cwiseProduct(v1).sum()) * sqrt(v2.cwiseProduct(v2).sum());
    // Judge duplicate points
    if (vector_dist < DBL_EPSILON) {
      continue;
    }
    double cos_theta = (v1.cwiseProduct(v2)).sum() / vector_dist;
    if (cos_theta > 1.0) {
      cos_theta = 1.0;
    } else if (cos_theta < -1.0) {
      cos_theta = -1.0;
    }
    double angle = (acos(cos_theta) * radian_to_degree);
    acos_theta += angle;
    if ((acos_theta - 1.0) > DBL_EPSILON) {
      polygon_ptr->push_back(point_1);
      spt = static_cast<unsigned int>(idx - 1);
      acos_theta = 0.0;
    }
  }
  // The last point
  polygon_ptr->push_back(raw_cloud[raw_cloud_size - 1]);
  AINFO << "Downsample road boundary points from " << raw_cloud_size << " to "
        << polygon_ptr->size();
}

void HDMapInput::SplitBoundary(
    const base::PointCloud<base::PointD>& boundary_line,
    const std::vector<base::PointCloud<base::PointD>>& junctions,
    std::vector<base::PointCloud<base::PointD>>* boundary_line_vec_ptr) {
  std::vector<bool> boundary_flag(boundary_line.size());
  for (size_t npt = 0; npt < boundary_line.size(); ++npt) {
    const PointD& pointd = boundary_line[npt];
    boundary_flag[npt] = false;
    for (size_t n_rj = 0; n_rj < junctions.size(); ++n_rj) {
      if (common::IsPointXYInPolygon2DXY(pointd, junctions[n_rj])) {
        boundary_flag[npt] = true;
        break;
      }
    }
  }
  std::vector<int> line_index;
  base::PolygonDType temp_line;
  for (size_t i = 1; i < boundary_flag.size(); ++i) {
    if (!boundary_flag[i - 1] || !boundary_flag[i]) {
      line_index.push_back(static_cast<int>(i - 1));
      line_index.push_back(static_cast<int>(i));
    } else if (line_index.size() > 1) {
      auto pos = std::unique(line_index.begin(), line_index.end());
      line_index.erase(pos, line_index.end());
      for (size_t j = 0; j < line_index.size(); ++j) {
        const PointD& pointd = boundary_line[line_index[j]];
        temp_line.push_back(pointd);
      }
      boundary_line_vec_ptr->push_back(temp_line);
      line_index.clear();
      temp_line.clear();
    }
  }
  // In case the last several unjunctioned points in the "boundary_line"
  if (line_index.size() > 1) {
    auto pos = std::unique(line_index.begin(), line_index.end());
    line_index.erase(pos, line_index.end());
    for (size_t j = 0; j < line_index.size(); ++j) {
      const PointD& pointd = boundary_line[line_index[j]];
      temp_line.push_back(pointd);
    }
    boundary_line_vec_ptr->push_back(temp_line);
  }
}

bool HDMapInput::GetNearestLaneDirection(const base::PointD& pointd,
                                         Eigen::Vector3d* lane_direction) {
  // if (hdmap_ == nullptr) {
  //   return false;
  // }
  // apollo::common::PointENU point;
  // point.set_x(pointd.x);
  // point.set_y(pointd.y);
  // point.set_z(pointd.z);
  // apollo::hdmap::LaneInfoConstPtr nearest_lane;
  // double nearest_s = 0.0;
  // double nearest_l = 0.0;
  // // get nearest lane of query point
  // int status = hdmap_->GetNearestLane(point, &nearest_lane,
  //     &nearest_s, &nearest_l);
  // if (status != 0) {
  //   AINFO << "Failed to get nearest lane for point " <<
  //   point.DebugString();
  //   return false;
  // }
  // // get lane heading of nearest s
  // const adu::hdmap::Trajectory& nearest_lane_trajectory
  //   = nearest_lane->trajectory();
  // double lane_heading
  //   = nearest_lane_trajectory.get_smooth_point(nearest_s).heading();
  // *lane_direction = Eigen::Vector3d(cos(lane_heading), sin(lane_heading), 0);
  return true;
}

bool HDMapInput::GetSignalsFromHDMap(
    const Eigen::Vector3d& pointd, double forward_distance,
    std::vector<apollo::hdmap::Signal>* signals) {
  apollo::common::PointENU point;
  point.set_x(pointd(0));
  point.set_y(pointd(1));
  point.set_z(pointd(2));
  std::vector<SignalInfoConstPtr> forward_signals;
  if (hdmap_->GetForwardNearestSignalsOnLane(point, forward_distance,
                                             &forward_signals) != 0) {
    AERROR << "Failed to call HDMap::get_signal. point: "
           << point.ShortDebugString();
    return false;
  }
  signals->reserve(forward_signals.size());
  for (auto& signal_info : forward_signals) {
    signals->push_back(signal_info->signal());
    ADEBUG << "Signal: " << signals->back().DebugString();
  }
  ADEBUG << "get_signal success. num_signals: " << signals->size()
         << " point: " << point.ShortDebugString();
  return true;
}

bool HDMapInput::GetSignals(const Eigen::Vector3d& pointd,
                            double forward_distance,
                            std::vector<apollo::hdmap::Signal>* signals) {
  lib::MutexLock lock(&mutex_);
  if (hdmap_.get() == nullptr) {
    AERROR << "hdmap is not available";
    return false;
  }
  return GetSignalsFromHDMap(pointd, forward_distance, signals);
}

}  // namespace map
}  // namespace perception
}  // namespace apollo
