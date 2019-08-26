/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <set>
#include <string>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/util/object.h"
#include "modules/perception/tool/benchmark/lidar/util/types.h"

namespace apollo {
namespace perception {
namespace benchmark {

class Frame : protected SensorObjects {
 public:
  Frame() : SensorObjects() { type = VELODYNE_64; }

  // Four elements should be loaded
  // 1. point cloud: _point_cloud
  // 2. result_objects: objects
  // 3. ground truth objects: gt_objects
  // 4. pose: sensor2world_pose
  bool load(const std::vector<std::string>& filenames);

  inline std::string get_name() const { return name; }

  inline const PointCloudConstPtr get_point_cloud() const {
    return _point_cloud;
  }

  inline const std::vector<ObjectPtr>& get_objects() const { return objects; }

  inline const std::vector<ObjectPtr>& get_gt_objects() const {
    return gt_objects;
  }

  inline const std::vector<std::vector<Eigen::Vector3d>>&
  get_objects_box_vertices() const {
    return objects_box_vertices;
  }

  inline const std::vector<std::vector<Eigen::Vector3d>>&
  get_gt_objects_box_vertices() const {
    return gt_objects_box_vertices;
  }
  inline const std::vector<PointCloud>& get_left_boundary() const {
    return _left_boundary;
  }
  inline const std::vector<PointCloud>& get_right_boundary() const {
    return _right_boundary;
  }
  inline const std::vector<PointCloud>& get_left_lane_boundary() const {
    return _left_lane_boundary;
  }
  inline const std::vector<PointCloud>& get_right_lane_boundary() const {
    return _right_lane_boundary;
  }
  inline const std::vector<PointCloud>& get_road_polygon() const {
    return _road_polygon;
  }
  inline void release() {
    _point_cloud = nullptr;
    objects.clear();
    gt_objects.clear();
    _left_boundary.clear();
    _right_boundary.clear();
    _left_lane_boundary.clear();
    _right_lane_boundary.clear();
    _road_polygon.clear();
  }

  static void set_black_list(const std::set<std::string>& black_list);

  static void set_is_for_visualization(bool for_visualization);

  static void set_visible_threshold(float threshold);

  static void set_min_confidence(float confidence);

 protected:
  // Two things should be done,
  // 1. transform points in objects to indices
  // 2. calculate points and indices in each gt objects if not exist;
  void build_indices();

  void build_points();

 private:
  void build_objects_indices(const pcl::KdTreeFLANN<Point>& point_cloud_kdtree,
                             std::vector<ObjectPtr>* objects_out);

  void build_objects_points(std::vector<ObjectPtr>* objects_out);

 protected:
  PointCloudPtr _point_cloud;
  std::vector<PointCloud> _left_boundary;
  std::vector<PointCloud> _right_boundary;
  std::vector<PointCloud> _left_lane_boundary;
  std::vector<PointCloud> _right_lane_boundary;
  std::vector<PointCloud> _road_polygon;
  std::vector<PointCloud> _lane_polygon;
  static std::set<std::string> _s_black_list;
  static bool _s_is_for_visualization;
  static float _s_distance_to_roi_boundary;
  static float _s_visible_threshold;
  static float _s_min_confidence;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
