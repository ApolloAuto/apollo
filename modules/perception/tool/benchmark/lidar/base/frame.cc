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

#include "modules/perception/tool/benchmark/lidar/base/frame.h"
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/util/geo_util.h"
#include "modules/perception/tool/benchmark/lidar/util/io_util.h"
#include "modules/perception/tool/benchmark/lidar/util/object_util.h"
#include "modules/perception/tool/benchmark/lidar/util/visibility.h"

namespace apollo {
namespace perception {
namespace benchmark {

std::set<std::string> Frame::_s_black_list;
bool Frame::_s_is_for_visualization = false;
float Frame::_s_distance_to_roi_boundary = 0.5f;
float Frame::_s_visible_threshold = 0.85f;
float Frame::_s_min_confidence = 0.0f;

void Frame::set_black_list(const std::set<std::string>& black_list) {
  _s_black_list = black_list;
}

void Frame::set_is_for_visualization(bool for_visualization) {
  _s_is_for_visualization = for_visualization;
}

void Frame::set_visible_threshold(float threshold) {
  _s_visible_threshold = threshold;
}

void Frame::set_min_confidence(float confidence) {
  _s_min_confidence = confidence;
}

bool Frame::load(const std::vector<std::string>& filenames) {
  if (filenames.size() < 3 || filenames.size() > 4) {
    std::cerr << "file list is not complete" << std::endl;
    return false;
  }
  name = filenames[0];
  // Step I: four elements should be loaded
  // 1. point cloud: _point_cloud
  // 2. result objects: objects
  // 3. ground truth objects: gt_objects
  // 4. pose: sensor2world_pose
  std::string pc_filename = filenames[0];
  std::string result_filename = filenames[1];
  std::string gt_filename = filenames[2];
  std::string pose_filename = "";
  if (filenames.size() == 4) {
    pose_filename = filenames[3];
    if (!load_sensor2world_pose(pose_filename, &sensor2world_pose)) {
      std::cerr << "Fail to load pose: " << pose_filename << std::endl;
      return false;
    }
  }
  _point_cloud.reset(new PointCloud);
  if (!load_pcl_pcds(pc_filename, _point_cloud)) {
    std::cerr << "Fail to load pcds: " << pc_filename << std::endl;
    return false;
  }
  std::vector<PointCloud>* left_boundary = &_left_boundary;
  std::vector<PointCloud>* right_boundary = &_right_boundary;
  std::vector<PointCloud>* road_polygon = &_road_polygon;
  std::vector<PointCloud>* left_lane_boundary = &_left_lane_boundary;
  std::vector<PointCloud>* right_lane_boundary = &_right_lane_boundary;

  PointCloud* cloud = _point_cloud.get();
  if (!load_frame_objects(result_filename,
                          std::set<std::string>(),  // _s_black_list,
                          &objects, left_boundary, right_boundary, road_polygon,
                          left_lane_boundary, right_lane_boundary, cloud)) {
    std::cerr << "Fail to load result: " << result_filename << std::endl;
    return false;
  }
  if (!load_frame_objects(gt_filename, _s_black_list, &gt_objects)) {
    std::cerr << "Fail to load groundtruth: " << gt_filename << std::endl;
    return false;
  }
  // confidence filter
  size_t valid = 0;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->confidence >= _s_min_confidence) {
      if (valid != i) {
        objects[valid] = objects[i];
      }
      ++valid;
    }
  }
  objects.resize(valid);

  // Step II: build indices easy for benchmark evaluation
  build_indices();

  // Step III: build points through indices
  build_points();

  // Step IV: precompute box vertices for each object
  objects_box_vertices.clear();
  objects_box_vertices.resize(objects.size());
  gt_objects_box_vertices.clear();
  gt_objects_box_vertices.resize(gt_objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->width < 1e-3 || objects[i]->length < 1e-3 ||
        objects[i]->height < 1e-3) {
      fill_axis_align_box(objects[i]);
    }
    get_bbox_vertices(objects[i], &objects_box_vertices[i]);
  }
  for (size_t i = 0; i < gt_objects.size(); ++i) {
    if (gt_objects[i]->width < 1e-3 || gt_objects[i]->length < 1e-3 ||
        gt_objects[i]->height < 1e-3) {
      fill_axis_align_box(gt_objects[i]);
    }
    get_bbox_vertices(gt_objects[i], &gt_objects_box_vertices[i]);
  }
  auto is_obj_in_roi = [&](const std::vector<Eigen::Vector3d>& vertices,
                           const std::vector<PointCloud>& rois) {
    if (vertices.size() < 4) {
      return false;
    }
    Point pt;
    for (auto& roi : rois) {
      for (size_t i = 0; i < 4; ++i) {
        pt.x = static_cast<float>(vertices[i](0));
        pt.y = static_cast<float>(vertices[i](1));
        if (is_point_xy_in_polygon2d_xy(pt, roi, _s_distance_to_roi_boundary)) {
          return true;
        }
      }
    }
    return false;
  };
  // Step V: fill roi flag
  if (_road_polygon.size() > 0) {
    for (size_t i = 0; i < objects.size(); ++i) {
      objects[i]->is_in_roi =
          is_obj_in_roi(objects_box_vertices[i], _road_polygon);
    }
    for (size_t i = 0; i < gt_objects.size(); ++i) {
      gt_objects[i]->is_in_roi =
          is_obj_in_roi(gt_objects_box_vertices[i], _road_polygon);
    }
  }
  // Step VI: construct and fill lane flag
  if (_left_lane_boundary.size() > 0 && _right_lane_boundary.size() > 0 &&
      _left_lane_boundary.size() == _right_lane_boundary.size()) {
    _lane_polygon.resize(_left_lane_boundary.size());
    for (size_t i = 0; i < _left_lane_boundary.size(); ++i) {
      _lane_polygon[i].points.clear();
      _lane_polygon[i].points.insert(_lane_polygon[i].points.end(),
                                     _left_lane_boundary[i].points.begin(),
                                     _left_lane_boundary[i].points.end());
      _lane_polygon[i].points.insert(_lane_polygon[i].points.end(),
                                     _right_lane_boundary[i].points.rbegin(),
                                     _right_lane_boundary[i].points.rend());
    }
    _road_polygon = _lane_polygon;
    for (size_t i = 0; i < objects.size(); ++i) {
      objects[i]->is_in_main_lanes =
          is_obj_in_roi(objects_box_vertices[i], _lane_polygon);
    }
    for (size_t i = 0; i < gt_objects.size(); ++i) {
      gt_objects[i]->is_in_main_lanes =
          is_obj_in_roi(gt_objects_box_vertices[i], _lane_polygon);
    }
  }
  // Step VI: fill visible ratio flag
  thread_local Visibility visibility(500, 500);
  visibility.set_car_pos(Eigen::Vector3d(0, 0, 0));
  visibility.fill_objects(&objects, _s_visible_threshold);
  visibility.fill_objects(&gt_objects, _s_visible_threshold);

  return true;
}

void Frame::build_indices() {
  bool objects_has_indices = true;
  bool gt_objects_has_indices = true;
  if (objects.size() > 0) {
    objects_has_indices = objects[0]->indices->indices.size() > 0;
  }
  if (gt_objects.size() > 0) {
    gt_objects_has_indices = gt_objects[0]->indices->indices.size() > 0;
  }
  if (objects_has_indices && gt_objects_has_indices) {
    return;
  }
  pcl::KdTreeFLANN<Point> point_cloud_kdtree;
  point_cloud_kdtree.setInputCloud(_point_cloud);
  // Step I: build result objects' indices
  if (!objects_has_indices) {
    build_objects_indices(point_cloud_kdtree, &objects);
  }
  // Step II: build ground truth objects' indices
  if (!gt_objects_has_indices) {
    build_objects_indices(point_cloud_kdtree, &gt_objects);
  }
}

void Frame::build_points() {
  bool objects_has_points = true;
  bool gt_objects_has_points = true;
  if (objects.size() > 0) {
    objects_has_points = objects[0]->cloud->size() > 0;
  }
  if (gt_objects.size() > 0) {
    gt_objects_has_points = gt_objects[0]->cloud->size() > 0;
  }
  if (objects_has_points && gt_objects_has_points) {
    return;
  }
  // Step I: build result objects' points
  if (!objects_has_points) {
    build_objects_points(&objects);
  }
  // Step II: build ground truth objects' points
  if (!gt_objects_has_points) {
    build_objects_points(&gt_objects);
  }
}

void Frame::build_objects_indices(
    const pcl::KdTreeFLANN<Point>& point_cloud_kdtree,
    std::vector<ObjectPtr>* objects_out) {
  std::vector<int> k_indices;
  std::vector<float> k_sqrt_dist;
  int objects_num = static_cast<int>(objects_out->size());
  for (int i = 0; i < objects_num; ++i) {
    int pts_num = static_cast<int>(objects_out->at(i)->cloud->points.size());
    objects_out->at(i)->indices->indices.resize(pts_num);
    for (int j = 0; j < pts_num; ++j) {
      const Point& pt = objects_out->at(i)->cloud->points[j];
      Point query_pt;
      query_pt.x = pt.x;
      query_pt.y = pt.y;
      query_pt.z = pt.z;
      k_indices.resize(1);
      k_sqrt_dist.resize(1);
      point_cloud_kdtree.nearestKSearch(query_pt, 1, k_indices, k_sqrt_dist);
      int query_indice = k_indices[0];
      objects_out->at(i)->indices->indices[j] = query_indice;
    }
  }
}

void Frame::build_objects_points(std::vector<ObjectPtr>* objects_out) {
  int objects_num = static_cast<int>(objects_out->size());
  for (int i = 0; i < objects_num; ++i) {
    int pts_num = static_cast<int>(objects_out->at(i)->indices->indices.size());
    objects_out->at(i)->cloud->points.resize(pts_num);
    for (int j = 0; j < pts_num; ++j) {
      const int& pid = objects_out->at(i)->indices->indices[j];
      objects_out->at(i)->cloud->at(j) = _point_cloud->at(pid);
    }
  }
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
