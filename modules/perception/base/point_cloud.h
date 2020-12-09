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

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/base/point.h"

using apollo::common::EigenVector;
using apollo::common::EigenMap;

namespace apollo {
namespace perception {
namespace base {

template <class PointT>
class PointCloud {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  using PointType = PointT;
  // @brief default constructor
  PointCloud() = default;

  // @brief construct from input point cloud and specified indices
  PointCloud(const PointCloud<PointT>& pc, const PointIndices& indices) {
    CopyPointCloud(pc, indices);
  }
  PointCloud(const PointCloud<PointT>& pc, const std::vector<int>& indices) {
    CopyPointCloud(pc, indices);
  }
  // @brief construct given width and height for organized point cloud
  PointCloud(size_t width, size_t height, PointT point = PointT())
      : width_(width), height_(height) {
    points_.assign(width_ * height_, point);
  }

  // @brief destructor
  virtual ~PointCloud() = default;

  // @brief accessor of point via 2d indices for organized cloud,
  // @return nullptr for non-organized cloud
  inline const PointT* at(size_t col, size_t row) const {
    return IsOrganized() ? &(points_[row * width_ + col]) : nullptr;
  }
  inline PointT* at(size_t col, size_t row) {
    return IsOrganized() ? &(points_[row * width_ + col]) : nullptr;
  }
  inline const PointT* operator()(size_t col, size_t row) const {
    return IsOrganized() ? &(points_[row * width_ + col]) : nullptr;
  }
  inline PointT* operator()(size_t col, size_t row) {
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
  inline const PointT& operator[](size_t n) const { return points_[n]; }
  inline PointT& operator[](size_t n) { return points_[n]; }
  inline const PointT& at(size_t n) const { return points_[n]; }
  inline PointT& at(size_t n) { return points_[n]; }
  // @brief front accessor wrapper of vector
  inline const PointT& front() const { return points_.front(); }
  inline PointT& front() { return points_.front(); }
  // @brief back accessor wrapper of vector
  inline const PointT& back() const { return points_.back(); }
  inline PointT& back() { return points_.back(); }
  // @brief push_back function wrapper of vector
  inline virtual void push_back(const PointT& point) {
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
                        const PointCloud<PointT>& rhs) {
    if (id < points_.size() && rhs_id < rhs.points_.size()) {
      points_[id] = rhs.points_[rhs_id];
      return true;
    }
    return false;
  }
  // @brief copy point cloud
  inline void CopyPointCloud(const PointCloud<PointT>& rhs,
                             const PointIndices& indices) {
    CopyPointCloud(rhs, indices.indices);
  }
  template <typename IndexType>
  inline void CopyPointCloud(const PointCloud<PointT>& rhs,
                             const std::vector<IndexType>& indices) {
    width_ = indices.size();
    height_ = 1;
    points_.resize(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      points_[i] = rhs.points_[indices[i]];
    }
  }
  template <typename IndexType>
  inline void CopyPointCloudExclude(const PointCloud<PointT>& rhs,
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
  inline void SwapPointCloud(PointCloud<PointT>* rhs) {
    points_.swap(rhs->points_);
    std::swap(width_, rhs->width_);
    std::swap(height_, rhs->height_);
    std::swap(sensor_to_world_pose_, rhs->sensor_to_world_pose_);
    std::swap(timestamp_, rhs->timestamp_);
  }
  typedef typename std::vector<PointT>::iterator iterator;
  typedef typename std::vector<PointT>::const_iterator const_iterator;
  // @brief vector iterator
  inline iterator begin() { return points_.begin(); }
  inline iterator end() { return points_.end(); }
  inline const_iterator begin() const { return points_.begin(); }
  inline const_iterator end() const { return points_.end(); }
  typename std::vector<PointT>* mutable_points() { return &points_; }
  const typename std::vector<PointT>& points() const { return points_; }

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
  void RotatePointCloud(bool check_nan = false) {
    Eigen::Vector3d point3d;
    Eigen::Matrix3d rotation = sensor_to_world_pose_.linear();
    for (auto& point : points_) {
      if (!check_nan || (!std::isnan(point.x) && !std::isnan(point.y) &&
                         !std::isnan(point.z))) {
        point3d << point.x, point.y, point.z;
        point3d = rotation * point3d;
        point.x = static_cast<typename PointT::Type>(point3d(0));
        point.y = static_cast<typename PointT::Type>(point3d(1));
        point.z = static_cast<typename PointT::Type>(point3d(2));
      }
    }
    sensor_to_world_pose_.linear().setIdentity();
  }
  // @brief transform the point cloud, set the pose to identity
  void TransformPointCloud(bool check_nan = false) {
    Eigen::Vector3d point3d;
    for (auto& point : points_) {
      if (!check_nan || (!std::isnan(point.x) && !std::isnan(point.y) &&
                         !std::isnan(point.z))) {
        point3d << point.x, point.y, point.z;
        point3d = sensor_to_world_pose_ * point3d;
        point.x = static_cast<typename PointT::Type>(point3d(0));
        point.y = static_cast<typename PointT::Type>(point3d(1));
        point.z = static_cast<typename PointT::Type>(point3d(2));
      }
    }
    sensor_to_world_pose_.setIdentity();
  }

  // @brief transform the point cloud and save to another pc
  void TransformPointCloud(const Eigen::Affine3f& transform,
                           PointCloud<PointT>* out,
                           bool check_nan = false) const {
    Eigen::Vector3f point3f;
    PointT pt;
    for (auto& point : points_) {
      if (!check_nan || (!std::isnan(point.x) && !std::isnan(point.y) &&
                         !std::isnan(point.z))) {
        point3f << point.x, point.y, point.z;
        point3f = transform * point3f;
        pt.x = static_cast<typename PointT::Type>(point3f(0));
        pt.y = static_cast<typename PointT::Type>(point3f(1));
        pt.z = static_cast<typename PointT::Type>(point3f(2));
        out->push_back(pt);
      }
    }
  }
  // @brief check data member consistency
  virtual bool CheckConsistency() const { return true; }

 protected:
  std::vector<PointT> points_;
  size_t width_ = 0;
  size_t height_ = 0;

  Eigen::Affine3d sensor_to_world_pose_ = Eigen::Affine3d::Identity();
  double timestamp_ = 0.0;
};

// @brief Point cloud class split points and attributes storage
// for fast traversing
template <class PointT>
class AttributePointCloud : public PointCloud<PointT> {
 public:
  using PointCloud<PointT>::points_;
  using PointCloud<PointT>::width_;
  using PointCloud<PointT>::height_;
  using PointCloud<PointT>::IsOrganized;
  using PointCloud<PointT>::sensor_to_world_pose_;
  using PointCloud<PointT>::timestamp_;
  // @brief default constructor
  AttributePointCloud() = default;

  // @brief construct from input point cloud and specified indices
  AttributePointCloud(const AttributePointCloud<PointT>& pc,
                      const PointIndices& indices) {
    CopyPointCloud(pc, indices);
  }
  AttributePointCloud(const AttributePointCloud<PointT>& pc,
                      const std::vector<int>& indices) {
    CopyPointCloud(pc, indices);
  }
  // @brief construct given width and height for organized point cloud
  AttributePointCloud(const size_t width, const size_t height,
                      const PointT point = PointT()) {
    width_ = width;
    height_ = height;
    size_t size = width_ * height_;
    points_.assign(size, point);
    points_timestamp_.assign(size, 0.0);
    points_height_.assign(size, std::numeric_limits<float>::max());
    points_beam_id_.assign(size, -1);
    points_label_.assign(size, 0);
  }
  // @brief destructor
  virtual ~AttributePointCloud() = default;
  // @brief add points of input cloud, return the self cloud
  inline AttributePointCloud& operator+=(
      const AttributePointCloud<PointT>& rhs) {
    points_.insert(points_.end(), rhs.points_.begin(), rhs.points_.end());
    points_timestamp_.insert(points_timestamp_.end(),
                             rhs.points_timestamp_.begin(),
                             rhs.points_timestamp_.end());
    points_height_.insert(points_height_.end(), rhs.points_height_.begin(),
                          rhs.points_height_.end());
    points_beam_id_.insert(points_beam_id_.end(), rhs.points_beam_id_.begin(),
                           rhs.points_beam_id_.end());
    points_label_.insert(points_label_.end(), rhs.points_label_.begin(),
                         rhs.points_label_.end());
    width_ = width_ * height_ + rhs.width_ * rhs.height_;
    height_ = 1;
    return *this;
  }
  // @brief overrided reserve function wrapper of vector
  inline void reserve(const size_t size) override {
    points_.reserve(size);
    points_timestamp_.reserve(size);
    points_height_.reserve(size);
    points_beam_id_.reserve(size);
    points_label_.reserve(size);
  }
  // @brief overrided resize function wrapper of vector
  inline void resize(const size_t size) override {
    points_.resize(size);
    points_timestamp_.resize(size, 0.0);
    points_height_.resize(size, std::numeric_limits<float>::max());
    points_beam_id_.resize(size, -1);
    points_label_.resize(size, 0);
    if (size != width_ * height_) {
      width_ = size;
      height_ = 1;
    }
  }
  // @brief overrided push_back function wrapper of vector
  inline void push_back(const PointT& point) override {
    points_.push_back(point);
    points_timestamp_.push_back(0.0);
    points_height_.push_back(std::numeric_limits<float>::max());
    points_beam_id_.push_back(-1);
    points_label_.push_back(0);
    width_ = points_.size();
    height_ = 1;
  }
  inline void push_back(const PointT& point, double timestamp,
                        float height = std::numeric_limits<float>::max(),
                        int32_t beam_id = -1, uint8_t label = 0) {
    points_.push_back(point);
    points_timestamp_.push_back(timestamp);
    points_height_.push_back(height);
    points_beam_id_.push_back(beam_id);
    points_label_.push_back(label);
    width_ = points_.size();
    height_ = 1;
  }
  // @brief overrided clear function wrapper of vector
  inline void clear() override {
    points_.clear();
    points_timestamp_.clear();
    points_height_.clear();
    points_beam_id_.clear();
    points_label_.clear();
    width_ = height_ = 0;
  }
  // @brief overrided swap point given source and target id
  inline bool SwapPoint(const size_t source_id,
                        const size_t target_id) override {
    if (source_id < points_.size() && target_id < points_.size()) {
      std::swap(points_[source_id], points_[target_id]);
      std::swap(points_timestamp_[source_id], points_timestamp_[target_id]);
      std::swap(points_height_[source_id], points_height_[target_id]);
      std::swap(points_beam_id_[source_id], points_beam_id_[target_id]);
      std::swap(points_label_[source_id], points_label_[target_id]);
      width_ = points_.size();
      height_ = 1;
      return true;
    }
    return false;
  }
  // @brief copy point from another point cloud
  inline bool CopyPoint(const size_t id, const size_t rhs_id,
                        const AttributePointCloud<PointT>& rhs) {
    if (id < points_.size() && rhs_id < rhs.points_.size()) {
      points_[id] = rhs.points_[rhs_id];
      points_timestamp_[id] = rhs.points_timestamp_[rhs_id];
      points_height_[id] = rhs.points_height_[rhs_id];
      points_beam_id_[id] = rhs.points_beam_id_[rhs_id];
      points_label_[id] = rhs.points_label_[rhs_id];
      return true;
    }
    return false;
  }
  // @brief copy point cloud
  inline void CopyPointCloud(const AttributePointCloud<PointT>& rhs,
                             const PointIndices& indices) {
    CopyPointCloud(rhs, indices.indices);
  }
  template <typename IndexType>
  inline void CopyPointCloud(const AttributePointCloud<PointT>& rhs,
                             const std::vector<IndexType>& indices) {
    width_ = indices.size();
    height_ = 1;
    points_.resize(indices.size());
    points_timestamp_.resize(indices.size());
    points_height_.resize(indices.size());
    points_beam_id_.resize(indices.size());
    points_label_.resize(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      points_[i] = rhs.points_[indices[i]];
      points_timestamp_[i] = rhs.points_timestamp_[indices[i]];
      points_height_[i] = rhs.points_height_[indices[i]];
      points_beam_id_[i] = rhs.points_beam_id_[indices[i]];
      points_label_[i] = rhs.points_label_[indices[i]];
    }
  }

  // @brief swap point cloud
  inline void SwapPointCloud(AttributePointCloud<PointT>* rhs) {
    points_.swap(rhs->points_);
    std::swap(width_, rhs->width_);
    std::swap(height_, rhs->height_);
    std::swap(sensor_to_world_pose_, rhs->sensor_to_world_pose_);
    std::swap(timestamp_, rhs->timestamp_);
    points_timestamp_.swap(rhs->points_timestamp_);
    points_height_.swap(rhs->points_height_);
    points_beam_id_.swap(rhs->points_beam_id_);
    points_label_.swap(rhs->points_label_);
  }
  // @brief overrided check data member consistency
  bool CheckConsistency() const override {
    return ((points_.size() == points_timestamp_.size()) &&
            (points_.size() == points_height_.size()) &&
            (points_.size() == points_beam_id_.size()) &&
            (points_.size() == points_label_.size()));
  }

  size_t TransferToIndex(const size_t col, const size_t row) const {
    return row * width_ + col;
  }

  const std::vector<double>& points_timestamp() const {
    return points_timestamp_;
  }
  double points_timestamp(size_t i) const { return points_timestamp_[i]; }
  std::vector<double>* mutable_points_timestamp() { return &points_timestamp_; }

  const std::vector<float>& points_height() const { return points_height_; }
  float& points_height(size_t i) { return points_height_[i]; }
  const float& points_height(size_t i) const { return points_height_[i]; }
  void SetPointHeight(size_t i, size_t j, float height) {
    points_height_[i * width_ + j] = height;
  }
  void SetPointHeight(size_t i, float height) { points_height_[i] = height; }
  std::vector<float>* mutable_points_height() { return &points_height_; }

  const std::vector<int32_t>& points_beam_id() const { return points_beam_id_; }
  std::vector<int32_t>* mutable_points_beam_id() { return &points_beam_id_; }
  int32_t& points_beam_id(size_t i) { return points_beam_id_[i]; }
  const int32_t& points_beam_id(size_t i) const { return points_beam_id_[i]; }

  const std::vector<uint8_t>& points_label() const { return points_label_; }
  std::vector<uint8_t>* mutable_points_label() { return &points_label_; }

  uint8_t& points_label(size_t i) { return points_label_[i]; }
  const uint8_t& points_label(size_t i) const { return points_label_[i]; }

 protected:
  std::vector<double> points_timestamp_;
  std::vector<float> points_height_;
  std::vector<int32_t> points_beam_id_;
  std::vector<uint8_t> points_label_;
};

// typedef of point cloud indices
typedef std::shared_ptr<PointIndices> PointIndicesPtr;
typedef std::shared_ptr<const PointIndices> PointIndicesConstPtr;

// typedef of point cloud
typedef AttributePointCloud<PointF> PointFCloud;
typedef AttributePointCloud<PointD> PointDCloud;

typedef std::shared_ptr<PointFCloud> PointFCloudPtr;
typedef std::shared_ptr<const PointFCloud> PointFCloudConstPtr;

typedef std::shared_ptr<PointDCloud> PointDCloudPtr;
typedef std::shared_ptr<const PointDCloud> PointDCloudConstPtr;

// typedef of polygon
typedef PointCloud<PointF> PolygonFType;
typedef PointCloud<PointD> PolygonDType;

typedef std::shared_ptr<PolygonFType> PolygonFTypePtr;
typedef std::shared_ptr<const PolygonFType> PolygonFTypeConstPtr;

typedef std::shared_ptr<PolygonDType> PolygonDTypePtr;
typedef std::shared_ptr<const PolygonDType> PolygonDTypeConstPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo
