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

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen/Eigenvalues"

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

/**@brief The data structure of a single ndt map cell. */
class NdtMapSingleCell {
 public:
  /**@brief The default constructor. */
  NdtMapSingleCell();
  /**@brief Reset to default value. */
  void Reset();
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  size_t LoadBinary(const unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  size_t CreateBinary(unsigned char* buf, size_t buf_size) const;
  /**@brief Get the binary size of the object. */
  size_t GetBinarySize() const;
  /**@brief Overloading the assign operator. */
  NdtMapSingleCell& operator=(const NdtMapSingleCell& ref);

  /**@brief Combine two NdtMapSingleCell instances (Reduce). */
  static void Reduce(NdtMapSingleCell* cell, const NdtMapSingleCell& cell_new);

  /**@brief Add an sample to the single 3d map cell. */
  void AddSample(const float intensity, const float altitude,
                 const Eigen::Vector3f& centroid, bool is_road = false);

  /**@brief Merge two cells. */
  void MergeCell(const float intensity, const float intensity_var,
                 const unsigned int road_pt_count, const unsigned int count,
                 const Eigen::Vector3f& centroid,
                 const Eigen::Matrix3f& centroid_cov);

  void MergeCell(const NdtMapSingleCell& cell_new);

  void CentroidEigenSolver(const Eigen::Matrix3f& centroid_cov);

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
                const float resolution, const Eigen::Vector3f& centroid,
                bool is_road = false);

  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  size_t LoadBinary(const unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  size_t CreateBinary(unsigned char* buf, size_t buf_size) const;
  /**@brief Get the binary size of the object. */
  size_t GetBinarySize() const;

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
  virtual void Init(const BaseMapConfig& config);
  /**@brief Reset the matrix item to default value. */
  virtual void Reset();

  /**@brief Initialize the matrix with the size of rows and columns. */
  void Init(unsigned int rows, unsigned int cols);
  /**@brief Reset the matrix item to default value. */
  void Reset(unsigned int rows, unsigned int cols);
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  virtual size_t LoadBinary(const unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual size_t CreateBinary(unsigned char* buf, size_t buf_size) const;
  /**@brief Get the binary size of the object. */
  virtual size_t GetBinarySize() const;

  virtual bool GetIntensityImg(cv::Mat* intensity_img) const;

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

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
