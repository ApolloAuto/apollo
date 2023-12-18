/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Dense"

#include "cyber/common/log.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/point.h"

namespace apollo {
namespace perception {
namespace base {

template <class RadarPointT>
class RadarPointCloud {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  using PointType = RadarPointT;
  // @brief default constructor
  RadarPointCloud() = default;

  // @brief construct from input point cloud and specified indices
  RadarPointCloud(const RadarPointCloud<RadarPointT>& pc,
    const PointIndices& indices) {
    CopyRadarPointCloud(pc, indices);
  }
  RadarPointCloud(const RadarPointCloud<RadarPointT>& pc,
    const std::vector<int>& indices) {
    CopyRadarPointCloud(pc, indices);
  }
  // @brief construct given width and height for organized point cloud
  RadarPointCloud(
    size_t width,
    size_t height,
    RadarPointT point = RadarPointT())
      : width_(width), height_(height) {
    points_.assign(width_ * height_, point);
  }

  // @brief destructor
  virtual ~RadarPointCloud() = default;

  // @brief accessor of point via 2d indices for organized cloud,
  // @return nullptr for non-organized cloud
  inline const RadarPointT* at(size_t col, size_t row) const {
    return IsOrganized() ? &(points_[row * width_ + col]) : nullptr;
  }
  inline RadarPointT* at(size_t col, size_t row) {
    return IsOrganized() ? &(points_[row * width_ + col]) : nullptr;
  }
  inline const RadarPointT* operator()(size_t col, size_t row) const {
    return IsOrganized() ? &(points_[row * width_ + col]) : nullptr;
  }
  inline RadarPointT* operator()(size_t col, size_t row) {
    return IsOrganized() ? &(points_[row * width_ + col]) : nullptr;
  }
  // @brief whether the cloud is organized
  inline bool IsOrganized() const { return height_ > 1; }
  // @brief accessor of point cloud height
  inline size_t height() const { return height_; }
  // @brief accessor of point cloud width
  inline size_t width() const { return width_; }
  // @brief accessor of point size, wrapper of vector
  inline size_t size() const { return points_.size(); }
  // @brief reserve function wrapper of vector
  inline virtual void reserve(size_t size) { points_.reserve(size); }
  // @brief empty function wrapper of vector
  inline bool empty() const { return points_.empty(); }
  // @brief resize function wrapper of vector
  inline virtual void resize(size_t size) {
    points_.resize(size);
    if (size != width_ * height_) {
      width_ = size;
      height_ = 1;
    }
  }
  // @brief accessor of point via 1d index
  inline const RadarPointT& operator[](size_t n) const { return points_[n]; }
  inline RadarPointT& operator[](size_t n) { return points_[n]; }
  inline const RadarPointT& at(size_t n) const { return points_[n]; }
  inline RadarPointT& at(size_t n) { return points_[n]; }
  // @brief front accessor wrapper of vector
  inline const RadarPointT& front() const { return points_.front(); }
  inline RadarPointT& front() { return points_.front(); }
  // @brief back accessor wrapper of vector
  inline const RadarPointT& back() const { return points_.back(); }
  inline RadarPointT& back() { return points_.back(); }
  // @brief push_back function wrapper of vector
  inline virtual void push_back(const RadarPointT& point) {
    points_.push_back(point);
    width_ = points_.size();
    height_ = 1;
  }
  // @brief clear function wrapper of vector
  inline virtual void clear() {
    points_.clear();
    width_ = height_ = 0;
  }
  // @brief swap point given source and target id
  inline virtual bool SwapPoint(size_t source_id, size_t target_id) {
    if (source_id < points_.size() && target_id < points_.size()) {
      std::swap(points_[source_id], points_[target_id]);
      width_ = points_.size();
      height_ = 1;
      return true;
    }
    return false;
  }
  // @brief copy point from another point cloud
  inline bool CopyPoint(size_t id, size_t rhs_id,
                        const RadarPointCloud<RadarPointT>& rhs) {
    if (id < points_.size() && rhs_id < rhs.points_.size()) {
      points_[id] = rhs.points_[rhs_id];
      return true;
    }
    return false;
  }
  // @brief copy point cloud
  inline void CopyRadarPointCloud(const RadarPointCloud<RadarPointT>& rhs,
                             const PointIndices& indices) {
    CopyRadarPointCloud(rhs, indices.indices);
  }
  template <typename IndexType>
  inline void CopyRadarPointCloud(const RadarPointCloud<RadarPointT>& rhs,
                             const std::vector<IndexType>& indices) {
    width_ = indices.size();
    height_ = 1;
    points_.resize(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      points_[i] = rhs.points_[indices[i]];
    }
  }
  template <typename IndexType>
  inline void CopyRadarPointCloudExclude(
    const RadarPointCloud<RadarPointT>& rhs,
    const std::vector<IndexType>& indices) {
    width_ = indices.size();
    height_ = 1;
    points_.resize(rhs.size() - indices.size());
    std::vector<bool> mask(false, rhs.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      mask[indices[i]] = true;
    }
    for (size_t i = 0; i < rhs.size(); ++i) {
      if (!mask[i]) {
        points_.push_back(rhs.points_[i]);
      }
    }
  }

  // @brief swap point cloud
  inline void SwapRadarPointCloud(RadarPointCloud<RadarPointT>* rhs) {
    points_.swap(rhs->points_);
    std::swap(width_, rhs->width_);
    std::swap(height_, rhs->height_);
    std::swap(sensor_to_world_pose_, rhs->sensor_to_world_pose_);
    std::swap(timestamp_, rhs->timestamp_);
  }
  typedef typename std::vector<RadarPointT>::iterator iterator;
  typedef typename std::vector<RadarPointT>::const_iterator const_iterator;
  // @brief vector iterator
  inline iterator begin() { return points_.begin(); }
  inline iterator end() { return points_.end(); }
  inline const_iterator begin() const { return points_.begin(); }
  inline const_iterator end() const { return points_.end(); }
  typename std::vector<RadarPointT>* mutable_points() { return &points_; }
  const typename std::vector<RadarPointT>& points() const { return points_; }

  // @brief cloud timestamp setter
  void set_timestamp(const double timestamp) { timestamp_ = timestamp; }
  // @brief cloud timestamp getter
  double get_timestamp() { return timestamp_; }
  // @brief sensor to world pose setter
  void set_sensor_to_world_pose(const Eigen::Affine3d& sensor_to_world_pose) {
    sensor_to_world_pose_ = sensor_to_world_pose;
  }
  // @brief sensor to world pose getter
  const Eigen::Affine3d& sensor_to_world_pose() {
    return sensor_to_world_pose_;
  }
  // @brief rotate the point cloud and set rotation part of pose to identity
  void RotateRadarPointCloud(bool check_nan = false) {
    Eigen::Vector3d point3d;
    Eigen::Matrix3d rotation = sensor_to_world_pose_.linear();
    for (auto& point : points_) {
      if (!check_nan || (!std::isnan(point.x) && !std::isnan(point.y) &&
                         !std::isnan(point.z))) {
        point3d << point.x, point.y, point.z;
        point3d = rotation * point3d;
        point.x = static_cast<typename RadarPointT::Type>(point3d(0));
        point.y = static_cast<typename RadarPointT::Type>(point3d(1));
        point.z = static_cast<typename RadarPointT::Type>(point3d(2));
      }
    }
    sensor_to_world_pose_.linear().setIdentity();
  }
  // @brief transform the point cloud, set the pose to identity
  void TransformRadarPointCloud(bool check_nan = false) {
    Eigen::Vector3d point3d;
    for (auto& point : points_) {
      if (!check_nan || (!std::isnan(point.x) && !std::isnan(point.y) &&
                         !std::isnan(point.z))) {
        point3d << point.x, point.y, point.z;
        point3d = sensor_to_world_pose_ * point3d;
        point.x = static_cast<typename RadarPointT::Type>(point3d(0));
        point.y = static_cast<typename RadarPointT::Type>(point3d(1));
        point.z = static_cast<typename RadarPointT::Type>(point3d(2));
      }
    }
    sensor_to_world_pose_.setIdentity();
  }

  // @brief transform the point cloud and save to another pc
  void TransformRadarPointCloud(const Eigen::Affine3f& transform,
                           RadarPointCloud<RadarPointT>* out,
                           bool check_nan = false) const {
    Eigen::Vector3f point3f;
    RadarPointT pt;
    for (auto& point : points_) {
      if (!check_nan || (!std::isnan(point.x) && !std::isnan(point.y) &&
                         !std::isnan(point.z))) {
        point3f << point.x, point.y, point.z;
        point3f = transform * point3f;
        pt.x = static_cast<typename RadarPointT::Type>(point3f(0));
        pt.y = static_cast<typename RadarPointT::Type>(point3f(1));
        pt.z = static_cast<typename RadarPointT::Type>(point3f(2));
        out->push_back(pt);
      }
    }
  }
  // @brief check data member consistency
  virtual bool CheckConsistency() const { return true; }

 protected:
  std::vector<RadarPointT> points_;
  size_t width_ = 0;
  size_t height_ = 0;

  Eigen::Affine3d sensor_to_world_pose_ = Eigen::Affine3d::Identity();
  double timestamp_ = 0.0;
};

// @brief Point cloud class split points and attributes storage
// for fast traversing
template <class RadarPointT>
class AttributeRadarPointCloud : public RadarPointCloud<RadarPointT> {
 public:
  using RadarPointCloud<RadarPointT>::points_;
  using RadarPointCloud<RadarPointT>::width_;
  using RadarPointCloud<RadarPointT>::height_;
  using RadarPointCloud<RadarPointT>::IsOrganized;
  using RadarPointCloud<RadarPointT>::sensor_to_world_pose_;
  using RadarPointCloud<RadarPointT>::timestamp_;
  // @brief default constructor
  AttributeRadarPointCloud() = default;

  // @brief construct from input point cloud and specified indices
  AttributeRadarPointCloud(const AttributeRadarPointCloud<RadarPointT>& pc,
                      const PointIndices& indices) {
    CopyRadarPointCloud(pc, indices);
  }
  AttributeRadarPointCloud(const AttributeRadarPointCloud<RadarPointT>& pc,
                      const std::vector<int>& indices) {
    CopyRadarPointCloud(pc, indices);
  }
  // @brief construct given width and height for organized point cloud
  AttributeRadarPointCloud(const size_t width, const size_t height,
                      const RadarPointT point = RadarPointT()) {
    width_ = width;
    height_ = height;
    if (width_ > 0 && height_ > SIZE_MAX / width_) {
      AFATAL << "overflow detected.";
      exit(1);
    }
    size_t size = width_ * height_;
    points_.assign(size, point);
  }
  // @brief destructor
  virtual ~AttributeRadarPointCloud() = default;
  // @brief add points of input cloud, return the self cloud
  inline AttributeRadarPointCloud& operator+=(
      const AttributeRadarPointCloud<RadarPointT>& rhs) {
    points_.insert(points_.end(), rhs.points_.begin(), rhs.points_.end());
    width_ = width_ * height_ + rhs.width_ * rhs.height_;
    height_ = 1;
    return *this;
  }
  // @brief overrided reserve function wrapper of vector
  inline void reserve(const size_t size) override {
    points_.reserve(size);
  }
  // @brief overrided resize function wrapper of vector
  inline void resize(const size_t size) override {
    points_.resize(size);
    if (size != width_ * height_) {
      width_ = size;
      height_ = 1;
    }
  }
  // @brief overrided push_back function wrapper of vector
  inline void push_back(const RadarPointT& point) override {
    points_.push_back(point);
    width_ = points_.size();
    height_ = 1;
  }
  inline void push_back(const RadarPointT& point, double timestamp,
                        float height = std::numeric_limits<float>::max(),
                        int32_t beam_id = -1, uint8_t label = 0) {
    points_.push_back(point);
    width_ = points_.size();
    height_ = 1;
  }
  // @brief overrided clear function wrapper of vector
  inline void clear() override {
    points_.clear();
    width_ = height_ = 0;
  }
  // @brief overrided swap point given source and target id
  inline bool SwapPoint(const size_t source_id,
                        const size_t target_id) override {
    if (source_id < points_.size() && target_id < points_.size()) {
      std::swap(points_[source_id], points_[target_id]);
      width_ = points_.size();
      height_ = 1;
      return true;
    }
    return false;
  }
  // @brief copy point from another point cloud
  inline bool CopyPoint(const size_t id, const size_t rhs_id,
                        const AttributeRadarPointCloud<RadarPointT>& rhs) {
    if (id < points_.size() && rhs_id < rhs.points_.size()) {
      points_[id] = rhs.points_[rhs_id];
      return true;
    }
    return false;
  }
  // @brief copy point cloud
  inline void CopyRadarPointCloud(
    const AttributeRadarPointCloud<RadarPointT>& rhs,
    const PointIndices& indices) {
    CopyRadarPointCloud(rhs, indices.indices);
  }
  template <typename IndexType>
  inline void CopyRadarPointCloud(
    const AttributeRadarPointCloud<RadarPointT>& rhs,
    const std::vector<IndexType>& indices) {
    width_ = indices.size();
    height_ = 1;
    points_.resize(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      points_[i] = rhs.points_[indices[i]];
    }
  }

  // @brief swap point cloud
  inline void SwapRadarPointCloud(AttributeRadarPointCloud<RadarPointT>* rhs) {
    points_.swap(rhs->points_);
    std::swap(width_, rhs->width_);
    std::swap(height_, rhs->height_);
    std::swap(sensor_to_world_pose_, rhs->sensor_to_world_pose_);
    std::swap(timestamp_, rhs->timestamp_);
  }
  // @brief overrided check data member consistency
  bool CheckConsistency() const override {
    return true;
  }

  size_t TransferToIndex(const size_t col, const size_t row) const {
    return row * width_ + col;
  }
};

// typedef of point cloud indices
typedef std::shared_ptr<PointIndices> PointIndicesPtr;
typedef std::shared_ptr<const PointIndices> PointIndicesConstPtr;

// typedef of point cloud
typedef AttributeRadarPointCloud<RadarPointF> RadarPointFCloud;
typedef AttributeRadarPointCloud<RadarPointD> RadarPointDCloud;

typedef std::shared_ptr<RadarPointFCloud> RadarPointFCloudPtr;
typedef std::shared_ptr<const RadarPointFCloud> RadarPointFCloudConstPtr;

typedef std::shared_ptr<RadarPointDCloud> RadarPointDCloudPtr;
typedef std::shared_ptr<const RadarPointDCloud> RadarPointDCloudConstPtr;

// typedef of polygon
typedef RadarPointCloud<PointF> RadarPolygonFType;
typedef RadarPointCloud<PointD> RadarPolygonDType;

typedef std::shared_ptr<RadarPolygonFType> RadarPolygonFTypePtr;
typedef std::shared_ptr<const RadarPolygonFType> RadarPolygonFTypeConstPtr;

typedef std::shared_ptr<RadarPolygonDType> RadarPolygonDTypePtr;
typedef std::shared_ptr<const RadarPolygonDType> RadarPolygonDTypeConstPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo
