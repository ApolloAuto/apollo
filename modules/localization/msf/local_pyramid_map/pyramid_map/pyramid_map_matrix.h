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

#include <memory>
#include <vector>
#include "modules/localization/msf/common/util/rect2d.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_matrix.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/aligned_matrix.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

typedef AlignedMatrix<float> FloatMatrix;
typedef AlignedMatrix<unsigned int> UIntMatrix;

class PyramidMapMatrix : public BaseMapMatrix {
 public:
  PyramidMapMatrix();
  ~PyramidMapMatrix();
  explicit PyramidMapMatrix(const PyramidMapMatrix& map_matrix);

  virtual void Init(const BaseMapConfig& config);
  /**@brief Reset map cells data. */
  virtual void Reset();

  void Init(unsigned int rows, unsigned int cols, bool has_intensity = true,
            bool has_intensity_var = true, bool has_altitude = true,
            bool has_altitude_var = true, bool has_ground_altitude = true,
            bool has_count = true, bool has_ground_count = true,
            unsigned int resolution_num = 1, unsigned int ratio = 2);

  /**@brief Reset all of map cells data in a specific resolution level. */
  void Reset(unsigned int level);
  /**@brief Reset map cells data from start_id to end_id
   * in a specific resolution level */
  void ResetCells(unsigned int start_id, unsigned int end_id,
                  unsigned int level = 0);
  /**@brief Reset a map cell in a specific resolution level. */
  void ResetCell(unsigned int id, unsigned int level = 0);

  /**@brief Release all memory in PyramidMapMatrix. */
  void Clear();

  /**@brief get intensity image of node. */
  virtual bool GetIntensityImg(cv::Mat* intensity_img) const;
  bool GetIntensityImg(unsigned int level, cv::Mat* intensity_img) const;
  /**@brief get altitude image of node. */
  virtual bool GetAltitudeImg(cv::Mat* altitude_img) const;
  bool GetAltitudeImg(unsigned int level, cv::Mat* altitude_img) const;

  /**@brief Propagate the data from fine level to the coarse resolution by
   * check. */
  void BottomUpSafe();
  /**@brief Propagate the data from fine level to the coarse resolution by
   * check. only update count, intensity, intensity var and altitude
   */
  void BottomUpBase();

  PyramidMapMatrix& operator=(const PyramidMapMatrix& map_matrix);

  /**@brief Get an intensity value by check. */
  const float* GetIntensitySafe(unsigned int row, unsigned int col,
                                unsigned int level = 0) const;
  /**@brief Get an intensity variance value by check. */
  const float* GetIntensityVarSafe(unsigned int row, unsigned int col,
                                   unsigned int level = 0) const;
  /**@brief Get an altitude value by check. */
  const float* GetAltitudeSafe(unsigned int row, unsigned int col,
                               unsigned int level = 0) const;
  /**@brief Get an altitude variance value by check. */
  const float* GetAltitudeVarSafe(unsigned int row, unsigned int col,
                                  unsigned int level = 0) const;
  /**@brief Get an altitude ground value by check. */
  const float* GetGroundAltitudeSafe(unsigned int row, unsigned int col,
                                     unsigned int level = 0) const;
  /**@brief Get a count value by check. */
  const unsigned int* GetCountSafe(unsigned int row, unsigned int col,
                                   unsigned int level = 0) const;
  /**@brief Get a ground count value by check. */
  const unsigned int* GetGroundCountSafe(unsigned int row, unsigned int col,
                                         unsigned int level = 0) const;

  /**@brief Get cell values by check. */
  void GetMapCellSafe(float** intensity, float** intensity_var,
                      float** altitude, float** altitude_var,
                      float** ground_altitude, unsigned int** count,
                      unsigned int** ground_count, unsigned int row,
                      unsigned int col, unsigned int level = 0);

  FloatMatrix* GetIntensityMatrixSafe(unsigned int level = 0);
  FloatMatrix* GetIntensityVarMatrixSafe(unsigned int level = 0);
  FloatMatrix* GetAltitudeMatrixSafe(unsigned int level = 0);
  FloatMatrix* GetAltitudeVarMatrixSafe(unsigned int level = 0);
  FloatMatrix* GetGroundAltitudeMatrixSafe(unsigned int level = 0);
  UIntMatrix* GetCountMatrixSafe(unsigned int level = 0);
  UIntMatrix* GetGroundCountMatrixSafe(unsigned int level = 0);

  const FloatMatrix* GetIntensityMatrixSafe(unsigned int level = 0) const;
  const FloatMatrix* GetIntensityVarMatrixSafe(unsigned int level = 0) const;
  const FloatMatrix* GetAltitudeMatrixSafe(unsigned int level = 0) const;
  const FloatMatrix* GetAltitudeVarMatrixSafe(unsigned int level = 0) const;
  const FloatMatrix* GetGroundAltitudeMatrixSafe(unsigned int level = 0) const;
  const UIntMatrix* GetCountMatrixSafe(unsigned int level = 0) const;
  const UIntMatrix* GetGroundCountMatrixSafe(unsigned int level = 0) const;

  void SetIntensityMatrix(const float* input, unsigned int size,
                          unsigned int start_index, unsigned int level = 0);
  void SetIntensityVarMatrix(const float* input, unsigned int size,
                             unsigned int start_index, unsigned int level = 0);
  void SetAltitudeMatrix(const float* input, unsigned int size,
                         unsigned int start_index, unsigned int level = 0);
  void SetAltitudeVarMatrix(const float* input, unsigned int size,
                            unsigned int start_index, unsigned int level = 0);
  void SetGroundAltitudeMatrix(const float* input, unsigned int size,
                               unsigned int start_index,
                               unsigned int level = 0);
  void SetCountMatrix(const unsigned int* input, unsigned int size,
                      unsigned int start_index, unsigned int level = 0);
  void SetGroundCountMatrix(const unsigned int* input, unsigned int size,
                            unsigned int start_index, unsigned int level = 0);

  /**@brief set float matrix with a ROI.
   * type: intensity 0; intensity_var 1; altitude 2;
   *       altitude_var 3; ground_altitude 4;
   */
  void SetFloatMatrixRoi(const FloatMatrix* source_matrix,
                         const Rect2D<unsigned int>& source_roi,
                         const Rect2D<unsigned int>& target_roi,
                         unsigned int type, unsigned int level = 0);

  /**@brief set unsigned int matrix with a ROI.
   * type: count 0; ground cout 1;
   */
  void SetUintMatrixRoi(const UIntMatrix* source_matrix,
                        const Rect2D<unsigned int>& source_roi,
                        const Rect2D<unsigned int>& target_roi,
                        unsigned int type, unsigned int level = 0);

  /**@brief Set an intensity value by check. */
  void SetIntensitySafe(float intensity, unsigned int row, unsigned int col,
                        unsigned int level = 0);
  /**@brief Set an intensity variance value by check. */
  void SetIntensityVarSafe(float intensity_var, unsigned int row,
                           unsigned int col, unsigned int level = 0);
  /**@brief Set an altitude value by check. */
  void SetAltitudeSafe(float altitude, unsigned int row, unsigned int col,
                       unsigned int level = 0);
  /**@brief Set an altitude variance value by check. */
  void SetAltitudeVarSafe(float altitude_var, unsigned int row,
                          unsigned int col, unsigned int level = 0);
  /**@brief Set an altitude ground value by check. */
  void SetGroundAltitudeSafe(float ground_altitude, unsigned int row,
                             unsigned int col, unsigned int level = 0);
  /**@brief Set a count value by check. */
  void SetCountSafe(unsigned int count, unsigned int row, unsigned int col,
                    unsigned int level = 0);
  /**@brief Set a ground count value by check. */
  void SetGroundCountSafe(unsigned int ground_count, unsigned int row,
                          unsigned int col, unsigned int level = 0);

  /**@brief Set the several values by check.
   * @param <altitude> The altitude of the cell.
   * @param <intensity> The reflectance intensity.
   */
  void SetValueSafe(unsigned char intensity, float altitude, unsigned int row,
                    unsigned int col, unsigned int level = 0);

  /**@brief Merge the data from another map cell by check. */
  void MergeCellSafe(const float* intensity, const float* intensity_var,
                     const float* altitude, const float* altitude_var,
                     const float* ground_altitude, const unsigned int* count,
                     const unsigned int* ground_count, unsigned int row,
                     unsigned int col, unsigned int level);

  /**@brief Add sample to the map cell with check. */
  void AddSampleSafe(float intensity, float altitude, unsigned int row,
                     unsigned int col, unsigned int level);

  /**@brief Add ground sample to the map cell. */
  void AddGroundSample(float ground_altitude, unsigned int row,
                       unsigned int col, unsigned int level = 0);

  /**@brief Compute mean intensity. */
  double ComputeMeanIntensity(unsigned int level = 0);

  /**@brief Combine two PyramidMapMatrix instances (Reduce). */
  static void Reduce(std::shared_ptr<PyramidMapMatrix> cells,
                     const PyramidMapMatrix& new_cells, unsigned int level = 0,
                     unsigned int new_level = 0);

  inline bool HasIntensity() const;
  inline bool HasIntensityVar() const;
  inline bool HasAltitude() const;
  inline bool HasAltitudeVar() const;
  inline bool HasGroundAltitude() const;
  inline bool HasCount() const;
  inline bool HasGroundCount() const;

  /**@brief Get row number given the resolution by check. */
  inline unsigned int GetRowsSafe(unsigned int level = 0) const;

  /**@brief Get column number given the resolution by check. */
  inline unsigned int GetColsSafe(unsigned int level = 0) const;

  /**@brief Get row number given the resolution. */
  inline unsigned int GetRows(unsigned int level = 0) const;

  /**@brief Get column number given the resolution. */
  inline unsigned int GetCols(unsigned int level = 0) const;

  /**@brief Get number of resolution. */
  inline unsigned int GetResolutionNum() const;

  /**@brief Get the resolution ratio. */
  inline unsigned int GetResolutionRatio() const;

  /**@brief Add sample to the map cell.
   * only update count, intensity, intensity var and altitude
   */
  inline void AddSampleBase(float intensity, float altitude, unsigned int row,
                            unsigned int col, unsigned int level);

  /**@brief Merge the data from another map cell.
   * only merge count, intensity, intensity var and altitude
   */
  inline void MergeCellBase(const float* intensity, const float* intensity_var,
                            const float* altitude, const unsigned int* count,
                            unsigned int row, unsigned int col,
                            unsigned int level);

  /**@brief Get cell values.
   * just get count, intensity, intensity var and altitude
   */
  inline void GetMapCellBase(float** intensity, float** intensity_var,
                             float** altitude, unsigned int** count,
                             unsigned int row, unsigned int col,
                             unsigned int level = 0);

  /**@brief Get an intensity value without check. */
  inline const float* GetIntensity(unsigned int row, unsigned int col,
                                   unsigned int level = 0) const;
  /**@brief Get an intensity variance value by check. */
  inline const float* GetIntensityVar(unsigned int row, unsigned int col,
                                      unsigned int level = 0) const;
  /**@brief Get an altitude value without check. */
  inline const float* GetAltitude(unsigned int row, unsigned int col,
                                  unsigned int level = 0) const;
  /**@brief Get an altitude variance value without check. */
  inline const float* GetAltitudeVar(unsigned int row, unsigned int col,
                                     unsigned int level = 0) const;
  /**@brief Get an altitude ground value without check. */
  inline const float* GetGroundAltitude(unsigned int row, unsigned int col,
                                        unsigned int level = 0) const;
  /**@brief Get a count value without check. */
  inline const unsigned int* GetCount(unsigned int row, unsigned int col,
                                      unsigned int level = 0) const;
  /**@brief Get a ground count value without check. */
  inline const unsigned int* GetGroundCount(unsigned int row, unsigned int col,
                                            unsigned int level = 0) const;

  inline FloatMatrix* GetIntensityMatrix(unsigned int level = 0);
  inline FloatMatrix* GetIntensityVarMatrix(unsigned int level = 0);
  inline FloatMatrix* GetAltitudeMatrix(unsigned int level = 0);
  inline FloatMatrix* GetAltitudeVarMatrix(unsigned int level = 0);
  inline FloatMatrix* GetGroundAltitudeMatrix(unsigned int level = 0);
  inline UIntMatrix* GetCountMatrix(unsigned int level = 0);
  inline UIntMatrix* GetGroundCountMatrix(unsigned int level = 0);

  inline const FloatMatrix* GetIntensityMatrix(unsigned int level = 0) const;
  inline const FloatMatrix* GetIntensityVarMatrix(unsigned int level = 0) const;
  inline const FloatMatrix* GetAltitudeMatrix(unsigned int level = 0) const;
  inline const FloatMatrix* GetAltitudeVarMatrix(unsigned int level = 0) const;
  inline const FloatMatrix* GetGroundAltitudeMatrix(
      unsigned int level = 0) const;
  inline const UIntMatrix* GetCountMatrix(unsigned int level = 0) const;
  inline const UIntMatrix* GetGroundCountMatrix(unsigned int level = 0) const;

 private:
  /**@brief check legality of params. */
  bool CheckLegalityForGetData(unsigned int row, unsigned int col,
                               unsigned int level) const;

  bool CheckLegalityForSetData(unsigned int level, unsigned int start_id,
                               unsigned int size) const;
  bool CheckLegalityForSetDataRoi(unsigned int level,
                                  unsigned int source_matrix_rows,
                                  unsigned int source_matrix_cols,
                                  const Rect2D<unsigned int>& source_roi,
                                  const Rect2D<unsigned int>& target_roi) const;

 private:
  /**@brief The number of resolutions. */
  unsigned int resolution_num_ = 1;
  /**@brief The ratio between adjacent resolutions. */
  unsigned int ratio_ = 2;
  /**@brief The multi resolution rows. */
  std::vector<unsigned int> rows_mr_;
  /**@brief The multi resolution cols. */
  std::vector<unsigned int> cols_mr_;
  /**@brief The multiplier buffer for speed up. */
  std::vector<unsigned int> ratio_multiplier_;

  std::vector<FloatMatrix> intensity_matrixes_;
  std::vector<FloatMatrix> intensity_var_matrixes_;
  std::vector<FloatMatrix> altitude_matrixes_;
  std::vector<FloatMatrix> altitude_var_matrixes_;
  std::vector<FloatMatrix> ground_altitude_matrixes_;
  std::vector<UIntMatrix> count_matrixes_;
  std::vector<UIntMatrix> ground_count_matrixes_;

  /**@brief Memory allocation flag for intensity_matrixes_. */
  bool has_intensity_ = false;
  /**@brief Memory allocation flag for intensity_var_matrixes_. */
  bool has_intensity_var_ = false;
  /**@brief Memory allocation flag for altitude_matrixes_. */
  bool has_altitude_ = false;
  /**@brief Memory allocation flag for altitude_var_matrixes_. */
  bool has_altitude_var_ = false;
  /**@brief Memory allocation flag for ground_altitude_matrixes_. */
  bool has_ground_altitude_ = false;
  /**@brief Memory allocation flag for count_matrixes_. */
  bool has_count_ = false;
  /**@brief Memory allocation flag for count_matrixes_. */
  bool has_ground_count_ = false;
};

inline bool PyramidMapMatrix::HasIntensity() const { return has_intensity_; }

inline bool PyramidMapMatrix::HasIntensityVar() const {
  return has_intensity_var_;
}

inline bool PyramidMapMatrix::HasAltitude() const { return has_altitude_; }

inline bool PyramidMapMatrix::HasAltitudeVar() const {
  return has_altitude_var_;
}

inline bool PyramidMapMatrix::HasGroundAltitude() const {
  return has_ground_altitude_;
}

inline bool PyramidMapMatrix::HasCount() const { return has_count_; }

inline bool PyramidMapMatrix::HasGroundCount() const {
  return has_ground_count_;
}

inline unsigned int PyramidMapMatrix::GetRowsSafe(unsigned int level) const {
  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetRows] The level id is illegal."
              << std::endl;
    return 0;
  }

  return rows_mr_[level];
}

inline unsigned int PyramidMapMatrix::GetColsSafe(unsigned int level) const {
  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetCols] The level id is illegal."
              << std::endl;
    return 0;
  }

  return cols_mr_[level];
}

inline unsigned int PyramidMapMatrix::GetRows(unsigned int level) const {
  return rows_mr_[level];
}

inline unsigned int PyramidMapMatrix::GetCols(unsigned int level) const {
  return cols_mr_[level];
}

inline unsigned int PyramidMapMatrix::GetResolutionNum() const {
  return resolution_num_;
}

inline unsigned int PyramidMapMatrix::GetResolutionRatio() const {
  return ratio_;
}

inline void PyramidMapMatrix::AddSampleBase(float intensity, float altitude,
                                            unsigned int row, unsigned int col,
                                            unsigned int level) {
  ++count_matrixes_[level][row][col];

  float v1 = 0.0;
  float v2 = 0.0;
  float value = 0.0;

  v1 = intensity - intensity_matrixes_[level][row][col];
  value = v1 / static_cast<float>(count_matrixes_[level][row][col]);
  intensity_matrixes_[level][row][col] += value;

  v2 = intensity - intensity_matrixes_[level][row][col];
  intensity_var_matrixes_[level][row][col] =
      (static_cast<float>(count_matrixes_[level][row][col] - 1) *
           intensity_var_matrixes_[level][row][col] +
       v1 * v2) /
      static_cast<float>(count_matrixes_[level][row][col]);

  v1 = altitude - altitude_matrixes_[level][row][col];
  value = v1 / static_cast<float>(count_matrixes_[level][row][col]);
  altitude_matrixes_[level][row][col] += value;
}

inline void PyramidMapMatrix::MergeCellBase(const float* intensity,
                                            const float* intensity_var,
                                            const float* altitude,
                                            const unsigned int* count,
                                            unsigned int row, unsigned int col,
                                            unsigned int level) {
  unsigned int new_count = count_matrixes_[level][row][col] + *count;
  float p0 = static_cast<float>(count_matrixes_[level][row][col]) /
             static_cast<float>(new_count);
  float p1 = static_cast<float>(*count) / static_cast<float>(new_count);

  float intensity_diff = 0.0f;
  intensity_diff = intensity_matrixes_[level][row][col] - *intensity;
  intensity_matrixes_[level][row][col] =
      intensity_matrixes_[level][row][col] * p0 + *intensity * p1;

  intensity_var_matrixes_[level][row][col] =
      intensity_var_matrixes_[level][row][col] * p0 + *intensity_var * p1 +
      intensity_diff * intensity_diff * p0 * p1;

  altitude_matrixes_[level][row][col] =
      altitude_matrixes_[level][row][col] * p0 + *altitude * p1;

  count_matrixes_[level][row][col] = new_count;
}

inline void PyramidMapMatrix::GetMapCellBase(float** intensity,
                                             float** intensity_var,
                                             float** altitude,
                                             unsigned int** count,
                                             unsigned int row, unsigned int col,
                                             unsigned int level) {
  *intensity = &intensity_matrixes_[level][row][col];
  *intensity_var = &intensity_var_matrixes_[level][row][col];
  *altitude = &altitude_matrixes_[level][row][col];
  *count = &count_matrixes_[level][row][col];
}

inline const float* PyramidMapMatrix::GetIntensity(unsigned int row,
                                                   unsigned int col,
                                                   unsigned int level) const {
  return &intensity_matrixes_[level][row][col];
}

inline const float* PyramidMapMatrix::GetIntensityVar(
    unsigned int row, unsigned int col, unsigned int level) const {
  return &intensity_var_matrixes_[level][row][col];
}

inline const float* PyramidMapMatrix::GetAltitude(unsigned int row,
                                                  unsigned int col,
                                                  unsigned int level) const {
  return &altitude_matrixes_[level][row][col];
}

inline const float* PyramidMapMatrix::GetAltitudeVar(unsigned int row,
                                                     unsigned int col,
                                                     unsigned int level) const {
  return &altitude_var_matrixes_[level][row][col];
}

inline const float* PyramidMapMatrix::GetGroundAltitude(
    unsigned int row, unsigned int col, unsigned int level) const {
  return &ground_altitude_matrixes_[level][row][col];
}

inline const unsigned int* PyramidMapMatrix::GetCount(
    unsigned int row, unsigned int col, unsigned int level) const {
  return &count_matrixes_[level][row][col];
}

inline const unsigned int* PyramidMapMatrix::GetGroundCount(
    unsigned int row, unsigned int col, unsigned int level) const {
  return &ground_count_matrixes_[level][row][col];
}

inline FloatMatrix* PyramidMapMatrix::GetIntensityMatrix(unsigned int level) {
  return &intensity_matrixes_[level];
}

inline FloatMatrix* PyramidMapMatrix::GetIntensityVarMatrix(
    unsigned int level) {
  return &intensity_var_matrixes_[level];
}

inline FloatMatrix* PyramidMapMatrix::GetAltitudeMatrix(unsigned int level) {
  return &altitude_matrixes_[level];
}

inline FloatMatrix* PyramidMapMatrix::GetAltitudeVarMatrix(unsigned int level) {
  return &altitude_var_matrixes_[level];
}

inline FloatMatrix* PyramidMapMatrix::GetGroundAltitudeMatrix(
    unsigned int level) {
  return &ground_altitude_matrixes_[level];
}

inline UIntMatrix* PyramidMapMatrix::GetCountMatrix(unsigned int level) {
  return &count_matrixes_[level];
}

inline UIntMatrix* PyramidMapMatrix::GetGroundCountMatrix(unsigned int level) {
  return &ground_count_matrixes_[level];
}

inline const FloatMatrix* PyramidMapMatrix::GetIntensityMatrix(
    unsigned int level) const {
  return &intensity_matrixes_[level];
}

inline const FloatMatrix* PyramidMapMatrix::GetIntensityVarMatrix(
    unsigned int level) const {
  return &intensity_var_matrixes_[level];
}

inline const FloatMatrix* PyramidMapMatrix::GetAltitudeMatrix(
    unsigned int level) const {
  return &altitude_matrixes_[level];
}

inline const FloatMatrix* PyramidMapMatrix::GetAltitudeVarMatrix(
    unsigned int level) const {
  return &altitude_var_matrixes_[level];
}

inline const FloatMatrix* PyramidMapMatrix::GetGroundAltitudeMatrix(
    unsigned int level) const {
  return &ground_altitude_matrixes_[level];
}

inline const UIntMatrix* PyramidMapMatrix::GetCountMatrix(
    unsigned int level) const {
  return &count_matrixes_[level];
}

inline const UIntMatrix* PyramidMapMatrix::GetGroundCountMatrix(
    unsigned int level) const {
  return &ground_count_matrixes_[level];
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
