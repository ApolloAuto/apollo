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
#ifndef PERCEPTION_FUSION_LIB_DATA_ASSOCIATION_PROJECTION_CACHE_H_
#define PERCEPTION_FUSION_LIB_DATA_ASSOCIATION_PROJECTION_CACHE_H_
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include "Eigen/StdVector"
#include "modules/perception/fusion/base/sensor_object.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d);

namespace apollo {
namespace perception {
namespace fusion {

// @brief: project cache of object
class ProjectionCacheObject {
 public:
  ProjectionCacheObject() : start_ind_(0), end_ind_(0) {
    box_ = base::BBox2DF();
  }
  explicit ProjectionCacheObject(size_t point2ds_size)
      : start_ind_(point2ds_size), end_ind_(point2ds_size) {
    box_ = base::BBox2DF();
  }
  // getters
  size_t GetStartInd() const { return start_ind_; }
  size_t GetEndInd() const { return end_ind_; }
  base::BBox2DF GetBox() const { return box_; }
  // setters
  void SetStartInd(size_t ind) { start_ind_ = ind; }
  void SetEndInd(size_t ind) { end_ind_ = ind; }
  void SetBox(base::BBox2DF box) { box_ = box; }
  const bool Empty() const { return (start_ind_ == end_ind_); }
  size_t Size() const { return (end_ind_ - start_ind_); }

 private:
  // project pts cache index of start/end, the pts of CacheObject belongs
  // to [start_ind_, end_ind_) of point2ds of Cache
  size_t start_ind_;
  size_t end_ind_;
  base::BBox2DF box_;
};  // class ProjectionCacheObject

// @brief: project cache of frame
class ProjectionCacheFrame {
 public:
  ProjectionCacheFrame() : sensor_id_(""), timestamp_(0.0) {}
  ProjectionCacheFrame(std::string sensor_id, double timestamp)
      : sensor_id_(sensor_id), timestamp_(timestamp) {}
  bool VerifyKey(std::string sensor_id, double timestamp) {
    if (sensor_id_ != sensor_id || fabs(timestamp_ - timestamp) > DBL_EPSILON) {
      return false;
    }
    return true;
  }
  void AddObject(int lidar_object_id, const ProjectionCacheObject& object) {
    objects_[lidar_object_id] = object;
  }
  bool QueryObject(int lidar_object_id, ProjectionCacheObject* object) {
    if (objects_.find(lidar_object_id) == objects_.end()) {
      return false;
    }
    *object = objects_[lidar_object_id];
    return true;
  }

 private:
  // sensor id of cached project frame
  std::string sensor_id_;
  double timestamp_;
  std::map<int, ProjectionCacheObject> objects_;
};  // class ProjectionCacheFrame

// @brief: project cache
class ProjectionCache {
 public:
  ProjectionCache() : measurement_sensor_id_(""), measurement_timestamp_(0.0) {
    // 300,000 pts is 2 times of the size of point cloud of ordinary frame of
    // velodyne64
    point2ds_.reserve(300000);
  }
  ProjectionCache(std::string sensor_id, double timestamp)
      : measurement_sensor_id_(sensor_id), measurement_timestamp_(timestamp) {
    // 300,000 pts is 2 times of the size of point cloud of ordinary frame of
    // velodyne64
    point2ds_.reserve(300000);
  }
  // reset projection cache
  void Reset(std::string sensor_id, double timestamp) {
    measurement_sensor_id_ = sensor_id;
    measurement_timestamp_ = timestamp;
    point2ds_.clear();
    frames_.clear();
  }
  // getters
  Eigen::Vector2d& GetPoint2d(size_t ind) { return point2ds_[ind]; }
  size_t GetPoint2dsSize() const { return point2ds_.size(); }
  // add point
  void AddPoint(const Eigen::Vector2f& pt) {
    point2ds_.emplace_back(pt.x(), pt.y());
  }
  // add object
  void AddObject(std::string projection_sensor_id, double projection_timestamp,
                 int lidar_object_id, const ProjectionCacheObject& object) {
    ProjectionCacheFrame frame =
        ProjectionCacheFrame(projection_sensor_id, projection_timestamp);
    if (!VerifyFrame(projection_sensor_id, projection_timestamp, &frame)) {
      frame.AddObject(lidar_object_id, object);
      frames_.push_back(frame);
      return;
    }
    for (size_t i = 0; i < frames_.size(); ++i) {
      if (!frames_[i].VerifyKey(projection_sensor_id, projection_timestamp)) {
        continue;
      }
      frames_[i].AddObject(lidar_object_id, object);
    }
  }
  // query projection cache object
  bool QueryObject(std::string measurement_sensor_id,
                   double measurement_timestamp,
                   std::string projection_sensor_id,
                   double projection_timestamp, int lidar_object_id,
                   ProjectionCacheObject* object) {
    if (!VerifyKey(measurement_sensor_id, measurement_timestamp)) {
      return false;
    }
    ProjectionCacheFrame frame = ProjectionCacheFrame();
    if (!VerifyFrame(projection_sensor_id, projection_timestamp, &frame)) {
      return false;
    }
    if (frame.QueryObject(lidar_object_id, object)) {
      return true;
    }
    return false;
  }

 private:
  bool VerifyKey(std::string sensor_id, double timestamp) {
    if (measurement_sensor_id_ != sensor_id ||
        fabs(measurement_timestamp_ - timestamp) > DBL_EPSILON) {
      return false;
    }
    return true;
  }
  bool VerifyFrame(std::string sensor_id, double timestamp,
                   ProjectionCacheFrame* frame) {
    for (size_t i = 0; i < frames_.size(); ++i) {
      if (!frames_[i].VerifyKey(sensor_id, timestamp)) continue;
      *frame = frames_[i];
      return true;
    }
    return false;
  }

 private:
  // sensor id & timestamp of measurement, which are the key of project cache
  std::string measurement_sensor_id_;
  double measurement_timestamp_;
  // project cache memeory
  std::vector<Eigen::Vector2d> point2ds_;
  // cache reference on frames
  std::vector<ProjectionCacheFrame> frames_;
};  // class ProjectionCache

typedef ProjectionCache* ProjectionCachePtr;

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_FUSION_LIB_DATA_ASSOCAITION_PROJECTION_CACHE_H_
