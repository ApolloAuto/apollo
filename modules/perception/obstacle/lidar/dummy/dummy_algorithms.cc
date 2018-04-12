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

#include "modules/perception/obstacle/lidar/dummy/dummy_algorithms.h"

#include <numeric>

#include "modules/perception/common/geometry_util.h"

namespace apollo {
namespace perception {

using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointD;
using pcl_util::PointIndices;
using pcl_util::PointIndicesPtr;

static void extract_pointcloud_indices(const PointCloudPtr &cloud,
                                       PointIndices *out_indices) {
  const size_t vec_size = cloud->size();
  auto &indices = out_indices->indices;
  indices.resize(vec_size);

  std::iota(indices.begin(), indices.end(), 0);
}

bool DummyROIFilter::Filter(const pcl_util::PointCloudPtr &cloud,
                            const ROIFilterOptions &roi_filter_options,
                            pcl_util::PointIndices *roi_indices) {
  extract_pointcloud_indices(cloud, roi_indices);
  return true;
}

bool DummyGroundDetector::Detect(const GroundDetectorOptions &options,
                                 PointCloudPtr cloud,
                                 PointIndicesPtr non_ground_indices) {
  extract_pointcloud_indices(cloud, non_ground_indices.get());
  return result_detect_;
}

bool DummySegmentation::Segment(const PointCloudPtr &cloud,
                                const PointIndices &non_ground_indices,
                                const SegmentationOptions &options,
                                std::vector<std::shared_ptr<Object>> *objects) {
  return result_segment_;
}

void DummyObjectBuilder::BuildObject(const ObjectBuilderOptions &options,
                                     std::shared_ptr<Object> obj) {
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  PointCloudPtr cloud = obj->cloud;
  SetDefaultValue(cloud, obj, &min_pt, &max_pt);
  if (cloud->points.size() < 4u) {
    return;
  }
  obj->polygon.points.resize(4);
  obj->polygon.points[0].x = static_cast<double>(min_pt[0]);
  obj->polygon.points[0].y = static_cast<double>(min_pt[1]);
  obj->polygon.points[0].z = static_cast<double>(min_pt[2]);

  obj->polygon.points[1].x = static_cast<double>(min_pt[0]);
  obj->polygon.points[1].y = static_cast<double>(max_pt[1]);
  obj->polygon.points[1].z = static_cast<double>(min_pt[2]);

  obj->polygon.points[2].x = static_cast<double>(max_pt[0]);
  obj->polygon.points[2].y = static_cast<double>(max_pt[1]);
  obj->polygon.points[2].z = static_cast<double>(min_pt[2]);

  obj->polygon.points[3].x = static_cast<double>(max_pt[0]);
  obj->polygon.points[3].y = static_cast<double>(min_pt[1]);
  obj->polygon.points[3].z = static_cast<double>(min_pt[2]);
  obj->anchor_point = obj->center;
}

bool DummyObjectBuilder::Build(const ObjectBuilderOptions &options,
                               std::vector<std::shared_ptr<Object>> *objects) {
  if (objects == NULL) {
    return false;
  }

  for (size_t i = 0; i < objects->size(); ++i) {
    if ((*objects)[i]) {
      (*objects)[i]->id = static_cast<int>(i);
      BuildObject(options, (*objects)[i]);
    }
  }

  return result_build_;
}

bool DummyObjectFilter::Filter(const ObjectFilterOptions &obj_filter_options,
                               std::vector<std::shared_ptr<Object>> *objects) {
  return result_object_filter_;
}

bool DummyTracker::Track(
    const std::vector<std::shared_ptr<Object>> &objects, double timestamp,
    const TrackerOptions &options,
    std::vector<std::shared_ptr<Object>> *tracked_objects) {
  if (tracked_objects == nullptr || options.velodyne_trans == nullptr) {
    return result_track_;
  }

  Eigen::Matrix4d pose = *(options.velodyne_trans);
  // transform objects
  (*tracked_objects).resize(objects.size());
  for (size_t i = 0; i < objects.size(); i++) {
    std::shared_ptr<Object> obj(new Object());
    obj->clone(*objects[i]);
    const Eigen::Vector3d &dir = obj->direction;
    obj->direction =
        (pose * Eigen::Vector4d(dir[0], dir[1], dir[2], 0)).head(3);
    const Eigen::Vector3d &center = obj->center;
    obj->center =
        (pose * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);
    // obj->anchor_point = obj->center;

    TransformPointCloud<pcl_util::Point>(pose, obj->cloud);
    TransformPointCloud<pcl_util::PointD>(pose, obj->polygon.makeShared());
    if (fabs(obj->direction[0]) < DBL_MIN) {
      if (obj->direction[1] > 0) {
        obj->theta = M_PI / 2;
      } else {
        obj->theta = -M_PI / 2;
      }
    } else {
      obj->theta = atan(obj->direction[1] / obj->direction[0]);
    }
    (*tracked_objects)[i] = obj;
  }

  return result_track_;
}

bool DummyTypeFuser::FuseType(const TypeFuserOptions &options,
                              std::vector<std::shared_ptr<Object>> *objects) {
  return result_type_fuser_;
}

}  // namespace perception
}  // namespace apollo
