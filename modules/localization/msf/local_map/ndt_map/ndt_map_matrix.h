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
#include <unordered_map>
#include <vector>

#include "Eigen/Eigenvalues"

#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of a single ndt map cell. */
class NdtMapSingleCell {
 public:
  /**@brief The default constructor. */
  NdtMapSingleCell();
  /**@brief Reset to default value. */
  inline void Reset();
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  unsigned int LoadBinary(unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  unsigned int CreateBinary(unsigned char* buf, unsigned int buf_size) const;
  /**@brief Get the binary size of the object. */
  unsigned int GetBinarySize() const;
  /**@brief Overloading the assign operator. */
  NdtMapSingleCell& operator=(const NdtMapSingleCell& ref);

  /**@brief Combine two NdtMapSingleCell instances (Reduce). */
  static void Reduce(NdtMapSingleCell* cell, const NdtMapSingleCell& cell_new);

  /**@brief Add an sample to the single 3d map cell. */
  inline void AddSample(const float intensity, const float altitude,
                        const Eigen::Vector3f centroid, bool is_road = false);

  /**@brief Merge two cells. */
  inline void MergeCell(const float intensity, const float intensity_var,
                        const unsigned int road_pt_count,
                        const unsigned int count,
                        const Eigen::Vector3f& centroid,
                        const Eigen::Matrix3f& centroid_cov);

  inline void MergeCell(const NdtMapSingleCell& cell_new);

  inline void CentroidEigenSolver(const Eigen::Matrix3f& centroid_cov);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /**@brief The average intensity value. */
  float intensity_ = 0;
  /**@brief The variance intensity value. */
  float intensity_var_ = 0;
  /**@brief The number of samples belonging to road surface. */
  unsigned int road_pt_count_ = 0;
  /**@brief The number of samples in the cell. */
  unsigned int count_ = 0;

  /**@brief the centroid of the cell. */
  Eigen::Vector3f centroid_;
  /**@brief the pose covariance of the cell. */
  Eigen::Matrix3f centroid_average_cov_;
  /**@brief the pose inverse covariance of the cell. */
  Eigen::Matrix3f centroid_icov_;
  /**@brief the inverse covariance available flag. */
  unsigned char is_icov_available_ = 0;
  /**@brief minimum number of points needed. */
  const unsigned int minimum_points_threshold_ = 6;
};

/**@brief The data structure of ndt Map cell. */
class NdtMapCells {
 public:
  /**@brief The default constructor. */
  NdtMapCells();
  /**@brief Reset to default value. */
  void Reset();
  /**@brief Add an sample. */
  int AddSample(const float intensity, const float altitude,
                const float resolution, const Eigen::Vector3f centroid,
                bool is_road = false);

  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  unsigned int LoadBinary(unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  unsigned int CreateBinary(unsigned char* buf, unsigned int buf_size) const;
  /**@brief Get the binary size of the object. */
  unsigned int GetBinarySize() const;

  /**@brief Calculate altitude index from altitude. */
  static int CalAltitudeIndex(const float resolution, const float altitude);

  /**@brief Calculate altitude from altitude index. */
  static float CalAltitude(const float resolution, const int altitude_index);

  /**@brief Combine two MapCell instances (Reduce). */
  static void Reduce(NdtMapCells* cell, const NdtMapCells& cell_new);

 public:
  /**@brief The multiple altitudes of the cell. */
  std::unordered_map<int, NdtMapSingleCell> cells_;
  /**@brief The index of biggest altitude. */
  int max_altitude_index_;
  /**@brief The index of smallest altitude. */
  int min_altitude_index_;
  /**@brief The indices of road surface. */
  std::vector<int> road_cell_indices_;
};

/**@brief The data structure of ndt Map matrix. */
class NdtMapMatrix : public BaseMapMatrix {
 public:
  /**@brief The default constructor. */
  NdtMapMatrix();
  /**@brief The default destructor. */
  ~NdtMapMatrix();
  /**@brief The copy constructor. */
  NdtMapMatrix(const NdtMapMatrix& cells);

  /**@brief Initialize the matrix with the config. */
  virtual void Init(const BaseMapConfig* config);
  /**@brief Reset the matrix item to default value. */
  virtual void Reset(const BaseMapConfig* config);

  /**@brief Initialize the matrix with the size of rows and columns. */
  void Init(unsigned int rows, unsigned int cols);
  /**@brief Reset the matrix item to default value. */
  void Reset(unsigned int rows, unsigned int cols);
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  virtual unsigned int LoadBinary(unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual unsigned int CreateBinary(unsigned char* buf,
                                    unsigned int buf_size) const;
  /**@brief Get the binary size of the object. */
  virtual unsigned int GetBinarySize() const;

  virtual void GetIntensityImg(cv::Mat* intensity_img) const;

  /**@brief Get a const map cell. */
  inline const NdtMapCells& GetMapCell(unsigned int row,
                                       unsigned int col) const {
    assert(row < rows_);
    assert(col < cols_);
    return map3d_cells_[row * cols_ + col];
  }
  /**@brief Get a map cell. */
  inline NdtMapCells& GetMapCell(unsigned int row, unsigned int col) {
    assert(row < rows_);
    assert(col < cols_);
    return map3d_cells_[row * cols_ + col];
  }
  /**@brief Get the size of row. */
  unsigned int GetRows() const { return rows_; }
  /**@brief Get the size of cols. */
  unsigned int GetCols() const { return cols_; }

  /**@brief Combine two NdtMapMatrix instances (Reduce). */
  static void Reduce(NdtMapMatrix* cells, const NdtMapMatrix& cells_new);

 private:
  /**@brief The number of rows. */
  unsigned int rows_;
  /**@brief The number of columns. */
  unsigned int cols_;
  /**@brief The matrix data structure. */
  std::unique_ptr<NdtMapCells[]> map3d_cells_;
};

inline void NdtMapSingleCell::Reset() {
  intensity_ = 0.0;
  intensity_var_ = 0.0;
  road_pt_count_ = 0;
  count_ = 0;
  centroid_ = Eigen::Vector3f::Zero();
  centroid_average_cov_ = Eigen::Matrix3f::Zero();
  centroid_icov_ = Eigen::Matrix3f::Identity();
  is_icov_available_ = false;
}

inline void NdtMapSingleCell::AddSample(const float intensity,
                                        const float altitude,
                                        const Eigen::Vector3f centroid,
                                        bool is_road) {
  ++count_;
  float v1 = intensity - intensity_;
  intensity_ += v1 / static_cast<float>(count_);
  float v2 = intensity - intensity_;
  intensity_var_ = (static_cast<float>(count_ - 1) * intensity_var_ + v1 * v2) /
                   static_cast<float>(count_);

  if (is_road) {
    ++road_pt_count_;
  }

  Eigen::Vector3f v3 = centroid - centroid_;
  centroid_ += v3 / static_cast<float>(count_);
  Eigen::Matrix3f v4 = centroid * centroid.transpose() - centroid_average_cov_;
  centroid_average_cov_ += v4 / static_cast<float>(count_);
  CentroidEigenSolver(centroid_average_cov_);
}

inline void NdtMapSingleCell::MergeCell(const float intensity,
                                        const float intensity_var,
                                        const unsigned int road_pt_count,
                                        const unsigned int count,
                                        const Eigen::Vector3f& centroid,
                                        const Eigen::Matrix3f& centroid_cov) {
  unsigned int new_count = count_ + count;
  float p0 = static_cast<float>(count_) / static_cast<float>(new_count);
  float p1 = static_cast<float>(count) / static_cast<float>(new_count);
  float intensity_diff = intensity_ - intensity;

  intensity_ = intensity_ * p0 + intensity * p1;
  intensity_var_ = intensity_var_ * p0 + intensity_var * p1 +
                   intensity_diff * intensity_diff * p0 * p1;

  centroid_[0] = centroid_[0] * p0 + centroid[0] * p1;
  centroid_[1] = centroid_[1] * p0 + centroid[1] * p1;
  centroid_[2] = centroid_[2] * p0 + centroid[2] * p1;

  count_ = new_count;
  road_pt_count_ += road_pt_count;
}

inline void NdtMapSingleCell::MergeCell(const NdtMapSingleCell& cell_new) {
  MergeCell(cell_new.intensity_, cell_new.intensity_var_,
            cell_new.road_pt_count_, cell_new.count_, cell_new.centroid_,
            cell_new.centroid_average_cov_);
}

inline void NdtMapSingleCell::CentroidEigenSolver(
    const Eigen::Matrix3f& centroid_cov) {
  // Contain more than five points, we calculate the eigen vector/value of cov.
  // [Magnusson 2009]
  if (count_ >= minimum_points_threshold_) {
    Eigen::Matrix3f cov = centroid_cov - centroid_ * centroid_.transpose();
    cov *= static_cast<float>((count_ - 1.0) / count_);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver;
    eigen_solver.compute(cov);
    Eigen::Matrix3f eigen_val = eigen_solver.eigenvalues().asDiagonal();
    Eigen::Matrix3f centroid_evecs = eigen_solver.eigenvectors();
    if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0) {
      is_icov_available_ = 0;
      return;
    }
    // Avoid matrices near singularities (eq 6.11) [Magnusson 2009].
    float min_covar_eigvalue = 0.01f * eigen_val(2, 2);
    if (eigen_val(0, 0) < min_covar_eigvalue) {
      eigen_val(0, 0) = min_covar_eigvalue;
      if (eigen_val(1, 1) < min_covar_eigvalue) {
        eigen_val(1, 1) = min_covar_eigvalue;
      }
    }
    // Calculate inverse covariance
    centroid_icov_ =
        (centroid_evecs * eigen_val * centroid_evecs.inverse()).inverse();
    if (centroid_icov_.maxCoeff() == std::numeric_limits<float>::infinity() ||
        centroid_icov_.minCoeff() == -std::numeric_limits<float>::infinity()) {
      is_icov_available_ = 0;
      return;
    }
    // Set icov available
    is_icov_available_ = 1;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
