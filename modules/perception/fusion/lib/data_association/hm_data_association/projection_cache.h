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
#pragma once

#include <map>
#include <string>
#include <vector>

#include "modules/common/math/math_utils.h"
#include "modules/perception/fusion/base/sensor_object.h"

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
    return sensor_id_ == sensor_id &&
           apollo::common::math::almost_equal(timestamp_, timestamp, 2);
  }
  ProjectionCacheObject* BuildObject(int lidar_object_id) {
    objects_[lidar_object_id] = ProjectionCacheObject();
    return QueryObject(lidar_object_id);
  }
  ProjectionCacheObject* QueryObject(int lidar_object_id) {
    auto it = objects_.find(lidar_object_id);
    if (it == objects_.end()) {
      return nullptr;
    } else {
      return &(it->second);
    }
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
  Eigen::Vector2d* GetPoint2d(size_t ind) {
    if (ind >= point2ds_.size()) {
      return nullptr;
    }
    return &(point2ds_[ind]);
  }
  size_t GetPoint2dsSize() const { return point2ds_.size(); }
  // add point
  void AddPoint(const Eigen::Vector2f& pt) {
    point2ds_.emplace_back(pt.x(), pt.y());
  }
  // add object
  ProjectionCacheObject* BuildObject(const std::string& measurement_sensor_id,
                                     double measurement_timestamp,
                                     const std::string& projection_sensor_id,
                                     double projection_timestamp,
                                     int lidar_object_id) {
    if (!VerifyKey(measurement_sensor_id, measurement_timestamp)) {
      return nullptr;
    }
    ProjectionCacheFrame* frame =
        QueryFrame(projection_sensor_id, projection_timestamp);
    if (frame == nullptr) {
      frame = BuildFrame(projection_sensor_id, projection_timestamp);
    }
    if (frame == nullptr) {
      return nullptr;
    }
    return frame->BuildObject(lidar_object_id);
  }
  // query projection cache object
  ProjectionCacheObject* QueryObject(const std::string& measurement_sensor_id,
                                     double measurement_timestamp,
                                     const std::string& projection_sensor_id,
                                     double projection_timestamp,
                                     int lidar_object_id) {
    if (!VerifyKey(measurement_sensor_id, measurement_timestamp)) {
      return nullptr;
    }
    ProjectionCacheFrame* frame =
        QueryFrame(projection_sensor_id, projection_timestamp);
    if (frame == nullptr) {
      return nullptr;
    }
    return frame->QueryObject(lidar_object_id);
  }

 private:
  bool VerifyKey(const std::string& sensor_id, double timestamp) {
    return measurement_sensor_id_ == sensor_id &&
           apollo::common::math::almost_equal(measurement_timestamp_, timestamp,
                                              2);
  }
  ProjectionCacheFrame* BuildFrame(const std::string& sensor_id,
                                   double timestamp) {
    frames_.push_back(ProjectionCacheFrame(sensor_id, timestamp));
    return &(frames_[frames_.size() - 1]);
  }
  ProjectionCacheFrame* QueryFrame(const std::string& sensor_id,
                                   double timestamp) {
    for (size_t i = 0; i < frames_.size(); ++i) {
      if (!frames_[i].VerifyKey(sensor_id, timestamp)) {
        continue;
      }
      return &(frames_[i]);
    }
    return nullptr;
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
