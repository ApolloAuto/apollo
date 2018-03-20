/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved. *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"

namespace apollo {
namespace perception {

bool HdmapROIFilter::Filter(const pcl_util::PointCloudPtr& cloud,
                            const ROIFilterOptions& roi_filter_options,
                            pcl_util::PointIndices* roi_indices) {
  if (roi_filter_options.hdmap == nullptr ||
      roi_filter_options.velodyne_trans == nullptr) {
    return false;
  }

  Eigen::Affine3d temp_trans(*(roi_filter_options.velodyne_trans));

  std::vector<PolygonDType> polygons;
  MergeHdmapStructToPolygons(roi_filter_options.hdmap, &polygons);

  if (polygons.size() == 0) {
    return false;
  }

  // 1. Transform polygon and point to local coordinates
  pcl_util::PointCloudPtr cloud_local(new pcl_util::PointCloud);
  std::vector<PolygonType> polygons_local;
  TransformFrame(cloud, temp_trans, polygons, &polygons_local, cloud_local);

  return FilterWithPolygonMask(cloud_local, polygons_local, roi_indices);
}

bool HdmapROIFilter::FilterWithPolygonMask(
    const pcl_util::PointCloudPtr& cloud,
    const std::vector<PolygonType>& map_polygons,
    pcl_util::PointIndices* roi_indices) {
  // 2. Get Major Direction as X direction and convert map_polygons to raw
  // polygons
  std::vector<PolygonScanConverter::Polygon> raw_polygons(map_polygons.size());
  MajorDirection major_dir = GetMajorDirection(map_polygons, &raw_polygons);

  // 3. Convert polygons into roi grids in bitmap
  Eigen::Vector2d min_p(-range_, -range_);
  Eigen::Vector2d max_p(range_, range_);
  Eigen::Vector2d grid_size(cell_size_, cell_size_);
  Bitmap2D bitmap(min_p, max_p, grid_size, major_dir);
  bitmap.BuildMap();

  DrawPolygonInBitmap(raw_polygons, extend_dist_, &bitmap);

  // 4. Check each point whether is in roi grids in bitmap
  return Bitmap2dFilter(cloud, bitmap, roi_indices);
}

MajorDirection HdmapROIFilter::GetMajorDirection(
    const std::vector<PolygonType>& map_polygons,
    std::vector<PolygonScanConverter::Polygon>* polygons) {
  double min_x = range_, min_y = range_;
  double max_x = -range_, max_y = -range_;
  auto& raw_polygons = *polygons;

  // Get available x_range and y_range, then set the direction with small range
  // as major direction.
  for (size_t i = 0; i < map_polygons.size(); ++i) {
    raw_polygons[i].resize(map_polygons[i].size());
    for (size_t j = 0; j < map_polygons[i].size(); ++j) {
      double x = map_polygons[i][j].x, y = map_polygons[i][j].y;
      raw_polygons[i][j].x() = x;
      raw_polygons[i][j].y() = y;

      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  }

  min_x = std::max(min_x, -range_);
  max_x = std::min(max_x, range_);
  min_y = std::max(min_y, -range_);
  max_y = std::min(max_y, range_);

  return (max_x - min_x) < (max_y - min_y) ? MajorDirection::XMAJOR
                                           : MajorDirection::YMAJOR;
}

bool HdmapROIFilter::Bitmap2dFilter(
    const pcl::PointCloud<pcl_util::Point>::ConstPtr in_cloud_ptr,
    const Bitmap2D& bitmap, pcl_util::PointIndices* roi_indices_ptr) {
  roi_indices_ptr->indices.reserve(in_cloud_ptr->size());
  for (size_t i = 0; i < in_cloud_ptr->size(); ++i) {
    const auto& pt = in_cloud_ptr->points[i];
    Eigen::Vector2d p(pt.x, pt.y);
    if (bitmap.IsExist(p) && bitmap.Check(p)) {
      roi_indices_ptr->indices.push_back(i);
    }
  }
  return true;
}

void HdmapROIFilter::MergeRoadBoundariesToPolygons(
    const std::vector<RoadBoundary>& road_boundaries,
    std::vector<PolygonDType>* polygons) {
  polygons->resize(road_boundaries.size());
  for (size_t i = 0; i < road_boundaries.size(); ++i) {
    // Assume the points of left boundary are [L1, L2, L3, ..., Ln],
    // points of right boundary are [R1, R2, R3, ..., Rn]. The polygon should be
    // like [L1, L2, L3, ..., Ln, Rn, ..., R3, R2, R1].
    const PolygonDType& left_boundary = road_boundaries[i].left_boundary;
    const PolygonDType& right_boundary = road_boundaries[i].right_boundary;

    auto& polygon = (*polygons)[i];
    polygon.reserve(left_boundary.size() + right_boundary.size());
    polygon.insert(polygon.end(), left_boundary.begin(), left_boundary.end());

    CHECK_GT(right_boundary.size(), 0);
    for (int j = static_cast<int>(right_boundary.size()) - 1; j >= 0; --j) {
      polygon.push_back(right_boundary[j]);
    }
  }
}

void HdmapROIFilter::MergeHdmapStructToPolygons(
    const HdmapStructConstPtr& hdmap_struct_ptr,
    std::vector<PolygonDType>* polygons) {
  std::vector<PolygonDType> road_polygons;
  MergeRoadBoundariesToPolygons(hdmap_struct_ptr->road_boundary,
                                &road_polygons);

  const std::vector<PolygonDType>& junction_polygons =
      hdmap_struct_ptr->junction;

  polygons->reserve(road_polygons.size() + junction_polygons.size());
  polygons->insert(polygons->end(), road_polygons.begin(), road_polygons.end());
  polygons->insert(polygons->end(), junction_polygons.begin(),
                   junction_polygons.end());
}

bool HdmapROIFilter::Init() {
  // load model config
  std::string model_name = name();
  const ModelConfig* model_config =
      ConfigManager::instance()->GetModelConfig(model_name);
  if (model_config == nullptr) {
    AERROR << "Failed to get model: " << model_name;
    return false;
  } else {
    if (!model_config->GetValue("range", &range_)) {
      AERROR << "Can not find range in model: " << model_name;
      return false;
    }
    if (!model_config->GetValue("cell_size", &cell_size_)) {
      AERROR << "Can not find cell_size in model: " << model_name;
      return false;
    }
    if (!model_config->GetValue("extend_dist", &extend_dist_)) {
      AERROR << "Can not find extend_dist_ in model: " << model_name;
      return false;
    }
  }
  return true;
}

void HdmapROIFilter::TransformFrame(
    const pcl_util::PointCloudConstPtr& cloud, const Eigen::Affine3d& vel_pose,
    const std::vector<PolygonDType>& polygons_world,
    std::vector<PolygonType>* polygons_local,
    pcl_util::PointCloudPtr cloud_local) {
  cloud_local->header = cloud->header;
  Eigen::Vector3d vel_location = vel_pose.translation();
  Eigen::Matrix3d vel_rot = vel_pose.linear();
  Eigen::Vector3d x_axis = vel_rot.row(0);
  Eigen::Vector3d y_axis = vel_rot.row(1);

  polygons_local->resize(polygons_world.size());
  for (size_t i = 0; i < polygons_local->size(); ++i) {
    const auto& polygon_world = polygons_world[i];
    auto& polygon_local = polygons_local->at(i);
    polygon_local.resize(polygon_world.size());
    for (size_t j = 0; j < polygon_local.size(); ++j) {
      polygon_local[j].x = polygon_world[j].x - vel_location.x();
      polygon_local[j].y = polygon_world[j].y - vel_location.y();
    }
  }

  cloud_local->resize(cloud->size());
  for (size_t i = 0; i < cloud_local->size(); ++i) {
    const auto& pt = cloud->points[i];
    auto& local_pt = cloud_local->points[i];
    Eigen::Vector3d e_pt(pt.x, pt.y, pt.z);
    local_pt.x = x_axis.dot(e_pt);
    local_pt.y = y_axis.dot(e_pt);
  }
}

}  // namespace perception
}  // namespace apollo
