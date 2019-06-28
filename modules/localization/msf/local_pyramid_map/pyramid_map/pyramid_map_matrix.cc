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
#include "modules/localization/msf/local_map/pyramid_map/pyramid_map_matrix.h"

#include <algorithm>

namespace apollo {
namespace localization {
namespace msf {

PyramidMapMatrix::PyramidMapMatrix() { Clear(); }

PyramidMapMatrix::~PyramidMapMatrix() { Clear(); }

PyramidMapMatrix::PyramidMapMatrix(const PyramidMapMatrix& map_matrix) {
  Clear();
  resolution_num_ = map_matrix.resolution_num_;
  ratio_ = map_matrix.ratio_;
  rows_mr_ = map_matrix.rows_mr_;
  cols_mr_ = map_matrix.cols_mr_;
  ratio_multiplier_ = map_matrix.ratio_multiplier_;

  intensity_matrixes_ = map_matrix.intensity_matrixes_;
  intensity_var_matrixes_ = map_matrix.intensity_var_matrixes_;
  altitude_matrixes_ = map_matrix.altitude_matrixes_;
  altitude_var_matrixes_ = map_matrix.altitude_var_matrixes_;
  ground_altitude_matrixes_ = map_matrix.ground_altitude_matrixes_;
  count_matrixes_ = map_matrix.count_matrixes_;
  ground_count_matrixes_ = map_matrix.ground_count_matrixes_;

  has_intensity_ = map_matrix.has_intensity_;
  has_intensity_var_ = map_matrix.has_intensity_var_;
  has_altitude_ = map_matrix.has_altitude_;
  has_altitude_var_ = map_matrix.has_altitude_var_;
  has_ground_altitude_ = map_matrix.has_ground_altitude_;
  has_count_ = map_matrix.has_count_;
  has_ground_count_ = map_matrix.has_ground_count_;
}

void PyramidMapMatrix::Init(const BaseMapConfig& config) {
  const PyramidMapConfig* pconfig =
      dynamic_cast<const PyramidMapConfig*>(&config);
  Init(pconfig->map_node_size_y_, pconfig->map_node_size_x_,
       pconfig->has_intensity_, pconfig->has_intensity_var_,
       pconfig->has_altitude_, pconfig->has_altitude_var_,
       pconfig->has_ground_altitude_, pconfig->has_count_,
       pconfig->has_ground_count_, pconfig->resolution_num_,
       pconfig->resolution_ratio_);
}

void PyramidMapMatrix::Reset() {
  for (unsigned int i = 0; i < resolution_num_; i++) {
    Reset(i);
  }
}

void PyramidMapMatrix::Init(unsigned int rows, unsigned int cols,
                            bool has_intensity, bool has_intensity_var,
                            bool has_altitude, bool has_altitude_var,
                            bool has_ground_altitude, bool has_count,
                            bool has_ground_count, unsigned int resolution_num,
                            unsigned int ratio) {
  Clear();

  // resolution_num should greater than 0
  if (resolution_num < 1) {
    std::cerr
        << "PyramidMapMatrix: [init] The resolution_num should greater than 0."
        << std::endl;
    return;
  }

  // ratio should greater than 0
  if (ratio < 1) {
    std::cerr << "PyramidMapMatrix: [init] The ratio should greater than 0."
              << std::endl;
    return;
  }

  // rows and cols in each level should be divisible by ratio
  unsigned int rows_tem = rows;
  unsigned int cols_tem = cols;
  for (unsigned int i = 1; i < resolution_num_; ++i) {
    unsigned int rows_remainder = rows_tem % ratio;
    unsigned int cols_remainder = cols_tem % ratio;

    if (rows_remainder != 0 || cols_remainder != 0) {
      std::cerr << "PyramidMapMatrix: [init] "
                << "Rows and cols in each level should be divisible by ratio."
                << std::endl;
      return;
    }

    rows_tem /= ratio;
    cols_tem /= ratio;
  }

  resolution_num_ = resolution_num;
  ratio_ = ratio;

  has_intensity_ = has_intensity;
  has_intensity_var_ = has_intensity_var;
  has_altitude_ = has_altitude;
  has_altitude_var_ = has_altitude_var;
  has_ground_altitude_ = has_ground_altitude;
  has_count_ = has_count;
  has_ground_count_ = has_ground_count;

  // Init rows_mr_ and cols_mr_
  rows_mr_.resize(resolution_num_, 0);
  cols_mr_.resize(resolution_num_, 0);
  rows_mr_[0] = rows;
  cols_mr_[0] = cols;
  for (unsigned int i = 1; i < resolution_num_; ++i) {
    rows_mr_[i] = rows_mr_[i - 1] / ratio_;
    cols_mr_[i] = cols_mr_[i - 1] / ratio_;
  }

  // Init data matrixes
  if (has_intensity_) {
    intensity_matrixes_.resize(resolution_num_);
    for (unsigned int i = 0; i < resolution_num_; i++) {
      intensity_matrixes_[i].Init(rows_mr_[i], cols_mr_[i]);
    }
  }
  if (has_intensity_var_) {
    intensity_var_matrixes_.resize(resolution_num_);
    for (unsigned int i = 0; i < resolution_num_; i++) {
      intensity_var_matrixes_[i].Init(rows_mr_[i], cols_mr_[i]);
    }
  }
  if (has_altitude_) {
    altitude_matrixes_.resize(resolution_num_);
    for (unsigned int i = 0; i < resolution_num_; i++) {
      altitude_matrixes_[i].Init(rows_mr_[i], cols_mr_[i]);
    }
  }
  if (has_altitude_var_) {
    altitude_var_matrixes_.resize(resolution_num_);
    for (unsigned int i = 0; i < resolution_num_; i++) {
      altitude_var_matrixes_[i].Init(rows_mr_[i], cols_mr_[i]);
    }
  }
  if (has_ground_altitude_) {
    ground_altitude_matrixes_.resize(resolution_num_);
    for (unsigned int i = 0; i < resolution_num_; i++) {
      ground_altitude_matrixes_[i].Init(rows_mr_[i], cols_mr_[i]);
    }
  }
  if (has_count_) {
    count_matrixes_.resize(resolution_num_);
    for (unsigned int i = 0; i < resolution_num_; i++) {
      count_matrixes_[i].Init(rows_mr_[i], cols_mr_[i]);
    }
  }
  if (has_ground_count_) {
    ground_count_matrixes_.resize(resolution_num_);
    for (unsigned int i = 0; i < resolution_num_; i++) {
      ground_count_matrixes_[i].Init(rows_mr_[i], cols_mr_[i]);
    }
  }

  // Init ratio multiplier
  unsigned int size = std::max(rows_mr_[0], cols_mr_[0]);
  ratio_multiplier_.resize(size, 0);
  for (unsigned int i = 0; i < size; ++i) {
    ratio_multiplier_[i] = i * ratio_;
  }
}

void PyramidMapMatrix::Reset(unsigned int level) {
  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [reset] The level id is illegal."
              << std::endl;
    return;
  }

  if (has_intensity_) {
    intensity_matrixes_[level].MakeEmpty();
  }
  if (has_intensity_var_) {
    intensity_var_matrixes_[level].MakeEmpty();
  }
  if (has_altitude_) {
    altitude_matrixes_[level].MakeEmpty();
  }
  if (has_altitude_var_) {
    altitude_var_matrixes_[level].MakeEmpty();
  }
  if (has_ground_altitude_) {
    ground_altitude_matrixes_[level].MakeEmpty();
  }
  if (has_count_) {
    count_matrixes_[level].MakeEmpty();
  }
  if (has_ground_count_) {
    ground_count_matrixes_[level].MakeEmpty();
  }
}

void PyramidMapMatrix::ResetCells(unsigned int start_id, unsigned int end_id,
                                  unsigned int level) {
  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [ResetCells] The level id is illegal."
              << std::endl;
    return;
  }

  unsigned int length = rows_mr_[level] * cols_mr_[level];
  if (start_id >= length || end_id >= length) {
    std::cerr
        << "PyramidMapMatrix: [ResetCell] The start_id or end_id is illegal."
        << std::endl;
    return;
  }

  if (has_intensity_) {
    intensity_matrixes_[level].MakeEmpty(start_id, end_id);
  }
  if (has_intensity_var_) {
    intensity_var_matrixes_[level].MakeEmpty(start_id, end_id);
  }
  if (has_altitude_) {
    altitude_matrixes_[level].MakeEmpty(start_id, end_id);
  }
  if (has_altitude_var_) {
    altitude_var_matrixes_[level].MakeEmpty(start_id, end_id);
  }
  if (has_ground_altitude_) {
    ground_altitude_matrixes_[level].MakeEmpty(start_id, end_id);
  }
  if (has_count_) {
    count_matrixes_[level].MakeEmpty(start_id, end_id);
  }
  if (has_ground_count_) {
    ground_count_matrixes_[level].MakeEmpty(start_id, end_id);
  }
}

void PyramidMapMatrix::ResetCell(unsigned int id, unsigned int level) {
  ResetCells(id, id, level);
}

void PyramidMapMatrix::Clear() {
  resolution_num_ = 1;
  ratio_ = 2;

  has_intensity_ = false;
  has_intensity_var_ = false;
  has_altitude_ = false;
  has_altitude_var_ = false;
  has_ground_altitude_ = false;
  has_count_ = false;
  has_ground_count_ = false;

  rows_mr_.clear();
  cols_mr_.clear();
  ratio_multiplier_.clear();
  intensity_matrixes_.clear();
  intensity_var_matrixes_.clear();
  altitude_matrixes_.clear();
  altitude_var_matrixes_.clear();
  ground_altitude_matrixes_.clear();
  count_matrixes_.clear();
  ground_count_matrixes_.clear();
}

bool PyramidMapMatrix::GetIntensityImg(cv::Mat* intensity_img) const {
  return GetIntensityImg(0, intensity_img);
}

bool PyramidMapMatrix::GetIntensityImg(unsigned int level,
                                       cv::Mat* intensity_img) const {
  if (!has_intensity_ || resolution_num_ < 1) {
    std::cerr << "PyramidMapMatrix: [GetIntensityImg] No intensity data."
              << std::endl;
    return false;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetIntensityImg] The level id is illegal."
              << std::endl;
    return false;
  }

  *intensity_img = cv::Mat(cv::Size(cols_mr_[level], rows_mr_[level]), CV_8UC1);

  for (unsigned int y = 0; y < rows_mr_[level]; ++y) {
    for (unsigned int x = 0; x < cols_mr_[level]; ++x) {
      if (intensity_matrixes_[level][y][x] < 0) {
        intensity_img->at<unsigned char>(y, x) = 0;
      } else if (intensity_matrixes_[level][y][x] > 255) {
        intensity_img->at<unsigned char>(y, x) = 255;
      } else {
        intensity_img->at<unsigned char>(y, x) =
            static_cast<unsigned char>(intensity_matrixes_[level][y][x]);
      }
    }
  }

  return true;
}

bool PyramidMapMatrix::GetAltitudeImg(cv::Mat* altitude_img) const {
  return GetAltitudeImg(0, altitude_img);
}

bool PyramidMapMatrix::GetAltitudeImg(unsigned int level,
                                      cv::Mat* altitude_img) const {
  if (!has_altitude_ || resolution_num_ < 1) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeImg] No altitude data."
              << std::endl;
    return false;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeImg] The level id is illegal."
              << std::endl;
    return false;
  }

  float min_alt = 1e8;
  float max_alt = -1e8;

  for (unsigned int y = 0; y < rows_mr_[level]; y++) {
    for (unsigned int x = 0; x < cols_mr_[level]; x++) {
      if (count_matrixes_[level][y][x] > 0) {
        if (altitude_matrixes_[level][y][x] > max_alt) {
          max_alt = altitude_matrixes_[level][y][x];
        }
        if (altitude_matrixes_[level][y][x] < min_alt) {
          min_alt = altitude_matrixes_[level][y][x];
        }
      }
    }
  }

  *altitude_img = cv::Mat(cv::Size(cols_mr_[level], rows_mr_[level]), CV_8UC1);
  for (unsigned int y = 0; y < rows_mr_[level]; y++) {
    for (unsigned int x = 0; x < cols_mr_[level]; x++) {
      if (count_matrixes_[level][y][x] > 0) {
        if (altitude_matrixes_[level][y][x] >= max_alt) {
          altitude_img->at<unsigned char>(y, x) = 255;
        } else if (altitude_matrixes_[level][y][x] <= min_alt) {
          altitude_img->at<unsigned char>(y, x) = 0;
        } else {
          altitude_img->at<unsigned char>(y, x) = static_cast<unsigned char>(
              (altitude_matrixes_[level][y][x] - min_alt) /
              (max_alt - min_alt) * 255);
        }
      } else {
        altitude_img->at<unsigned char>(y, x) = 0;
      }
    }
  }

  return true;
}

void PyramidMapMatrix::BottomUpSafe() {
  if (!has_count_) {
    std::cerr << "PyramidMapMatrix: [bottom_up] Has no count." << std::endl;
    return;
  }

  for (unsigned int i = 1; i < resolution_num_; ++i) {
    const unsigned int& row = rows_mr_[i];
    const unsigned int& col = rows_mr_[i];
    for (unsigned int r = 0; r < row; ++r) {
      for (unsigned int c = 0; c < col; ++c) {
        unsigned int& r0 = ratio_multiplier_[r];
        unsigned int& c0 = ratio_multiplier_[c];
        for (unsigned int rl = r0; rl < r0 + ratio_; ++rl) {
          for (unsigned int cl = c0; cl < c0 + ratio_; ++cl) {
            const unsigned int& count = count_matrixes_[i - 1][rl][cl];
            if (count > 0) {
              float* intensity = NULL;
              float* intensity_var = NULL;
              float* altitude = NULL;
              float* altitude_var = NULL;
              float* ground_altitude = NULL;
              unsigned int* count = NULL;
              unsigned int* ground_count = NULL;

              GetMapCellSafe(&intensity, &intensity_var, &altitude,
                             &altitude_var, &ground_altitude, &count,
                             &ground_count, rl, cl, i - 1);
              MergeCellSafe(intensity, intensity_var, altitude, altitude_var,
                            ground_altitude, count, ground_count, r, c, i);
            }
          }
        }
      }
    }
  }
}

void PyramidMapMatrix::BottomUpBase() {
  for (unsigned int i = 1; i < resolution_num_; ++i) {
    const unsigned int& row = rows_mr_[i];
    const unsigned int& col = rows_mr_[i];
    for (unsigned int r = 0; r < row; ++r) {
      for (unsigned int c = 0; c < col; ++c) {
        unsigned int& r0 = ratio_multiplier_[r];
        unsigned int& c0 = ratio_multiplier_[c];
        for (unsigned int rl = r0; rl < r0 + ratio_; ++rl) {
          for (unsigned int cl = c0; cl < c0 + ratio_; ++cl) {
            const unsigned int& count = count_matrixes_[i - 1][rl][cl];
            if (count > 0) {
              float* intensity = NULL;
              float* intensity_var = NULL;
              float* altitude = NULL;
              unsigned int* count = NULL;

              GetMapCellBase(&intensity, &intensity_var, &altitude, &count, rl,
                             cl, i - 1);
              MergeCellBase(intensity, intensity_var, altitude, count, r, c, i);
            }
          }
        }
      }
    }
  }
}

PyramidMapMatrix& PyramidMapMatrix::operator=(
    const PyramidMapMatrix& map_matrix) {
  Clear();
  resolution_num_ = map_matrix.resolution_num_;
  ratio_ = map_matrix.ratio_;
  rows_mr_ = map_matrix.rows_mr_;
  cols_mr_ = map_matrix.cols_mr_;
  ratio_multiplier_ = map_matrix.ratio_multiplier_;

  intensity_matrixes_ = map_matrix.intensity_matrixes_;
  intensity_var_matrixes_ = map_matrix.intensity_var_matrixes_;
  altitude_matrixes_ = map_matrix.altitude_matrixes_;
  altitude_var_matrixes_ = map_matrix.altitude_var_matrixes_;
  ground_altitude_matrixes_ = map_matrix.ground_altitude_matrixes_;
  count_matrixes_ = map_matrix.count_matrixes_;
  ground_count_matrixes_ = map_matrix.ground_count_matrixes_;

  has_intensity_ = map_matrix.has_intensity_;
  has_intensity_var_ = map_matrix.has_intensity_var_;
  has_altitude_ = map_matrix.has_altitude_;
  has_altitude_var_ = map_matrix.has_altitude_var_;
  has_ground_altitude_ = map_matrix.has_ground_altitude_;
  has_count_ = map_matrix.has_count_;
  has_ground_count_ = map_matrix.has_ground_count_;
  return *this;
}

const float* PyramidMapMatrix::GetIntensitySafe(unsigned int row,
                                                unsigned int col,
                                                unsigned int level) const {
  if (!has_intensity_) {
    std::cerr << "PyramidMapMatrix: [GetIntensitySafe] Has no intensity."
              << std::endl;
    return NULL;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [GetIntensitySafe] Params is illegal."
              << std::endl;
    return NULL;
  }

  return &intensity_matrixes_[level][row][col];
}

const float* PyramidMapMatrix::GetIntensityVarSafe(unsigned int row,
                                                   unsigned int col,
                                                   unsigned int level) const {
  if (!has_intensity_var_) {
    std::cerr << "PyramidMapMatrix: [GetIntensityVarSafe] Has no intensity_var."
              << std::endl;
    return NULL;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [GetIntensityVarSafe] Params is illegal."
              << std::endl;
    return NULL;
  }

  return &intensity_var_matrixes_[level][row][col];
}

const float* PyramidMapMatrix::GetAltitudeSafe(unsigned int row,
                                               unsigned int col,
                                               unsigned int level) const {
  if (!has_altitude_) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeSafe] Has no altitude."
              << std::endl;
    return NULL;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeSafe] Params is illegal."
              << std::endl;
    return NULL;
  }

  return &altitude_matrixes_[level][row][col];
}

const float* PyramidMapMatrix::GetAltitudeVarSafe(unsigned int row,
                                                  unsigned int col,
                                                  unsigned int level) const {
  if (!has_altitude_var_) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeVarSafe] Has no altitude_var."
              << std::endl;
    return NULL;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [get_altitude_var] Params is illegal."
              << std::endl;
    return NULL;
  }

  return &altitude_var_matrixes_[level][row][col];
}

const float* PyramidMapMatrix::GetGroundAltitudeSafe(unsigned int row,
                                                     unsigned int col,
                                                     unsigned int level) const {
  if (!has_ground_altitude_) {
    std::cerr
        << "PyramidMapMatrix: [GetGroundAltitudeSafe] Has no ground_altitude."
        << std::endl;
    return NULL;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [GetGroundAltitudeSafe] Params is illegal."
              << std::endl;
    return NULL;
  }

  return &ground_altitude_matrixes_[level][row][col];
}

const unsigned int* PyramidMapMatrix::GetCountSafe(unsigned int row,
                                                   unsigned int col,
                                                   unsigned int level) const {
  if (!has_count_) {
    std::cerr << "PyramidMapMatrix: [GetCountSafe] Has no count." << std::endl;
    return NULL;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [GetCountSafe] Params is illegal."
              << std::endl;
    return NULL;
  }

  return &count_matrixes_[level][row][col];
}

const unsigned int* PyramidMapMatrix::GetGroundCountSafe(
    unsigned int row, unsigned int col, unsigned int level) const {
  if (!has_ground_count_) {
    std::cerr << "PyramidMapMatrix: [GetGroundCountSafe] Has no ground_count."
              << std::endl;
    return NULL;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [GetGroundCountSafe] Params is illegal."
              << std::endl;
    return NULL;
  }

  return &ground_count_matrixes_[level][row][col];
}

void PyramidMapMatrix::GetMapCellSafe(float** intensity, float** intensity_var,
                                      float** altitude, float** altitude_var,
                                      float** ground_altitude,
                                      unsigned int** count,
                                      unsigned int** ground_count,
                                      unsigned int row, unsigned int col,
                                      unsigned int level) {
  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [GetMapCellSafe] Params is illegal."
              << std::endl;
    return;
  }

  if (has_intensity_) {
    *intensity = &intensity_matrixes_[level][row][col];
  }

  if (has_intensity_var_) {
    *intensity_var = &intensity_var_matrixes_[level][row][col];
  }

  if (has_altitude_) {
    *altitude = &altitude_matrixes_[level][row][col];
  }

  if (has_altitude_var_) {
    *altitude_var = &altitude_var_matrixes_[level][row][col];
  }

  if (has_ground_altitude_) {
    *ground_altitude = &ground_altitude_matrixes_[level][row][col];
  }

  if (has_count_) {
    *count = &count_matrixes_[level][row][col];
  }

  if (has_ground_count_) {
    *ground_count = &ground_count_matrixes_[level][row][col];
  }
}

FloatMatrix* PyramidMapMatrix::GetIntensityMatrixSafe(unsigned int level) {
  if (!has_intensity_) {
    std::cerr << "PyramidMapMatrix: [GetIntensityMatrixSafe] Has no intensity."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr
        << "PyramidMapMatrix: [GetIntensityMatrixSafe] The level id is illegal."
        << std::endl;
    return NULL;
  }

  return &intensity_matrixes_[level];
}

FloatMatrix* PyramidMapMatrix::GetIntensityVarMatrixSafe(unsigned int level) {
  if (!has_intensity_var_) {
    std::cerr
        << "PyramidMapMatrix: [GetIntensityVarMatrixSafe] Has no intensity_var."
        << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetIntensityVarMatrixSafe] The level id "
                 "is illegal."
              << std::endl;
    return NULL;
  }

  return &intensity_var_matrixes_[level];
}

FloatMatrix* PyramidMapMatrix::GetAltitudeMatrixSafe(unsigned int level) {
  if (!has_altitude_) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeMatrixSafe] Has no altitude."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr
        << "PyramidMapMatrix: [GetAltitudeMatrixSafe] The level id is illegal."
        << std::endl;
    return NULL;
  }

  return &altitude_matrixes_[level];
}

FloatMatrix* PyramidMapMatrix::GetAltitudeVarMatrixSafe(unsigned int level) {
  if (!has_altitude_var_) {
    std::cerr
        << "PyramidMapMatrix: [GetAltitudeVarMatrixSafe] Has no altitude_var."
        << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeVarMatrixSafe] The level id is "
                 "illegal."
              << std::endl;
    return NULL;
  }

  return &altitude_var_matrixes_[level];
}

FloatMatrix* PyramidMapMatrix::GetGroundAltitudeMatrixSafe(unsigned int level) {
  if (!has_ground_altitude_) {
    std::cerr << "PyramidMapMatrix: [GetGroundAltitudeMatrixSafe] Has no "
                 "ground_altitude."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetGroundAltitudeMatrixSafe] The level id "
                 "is illegal."
              << std::endl;
    return NULL;
  }

  return &ground_altitude_matrixes_[level];
}

UIntMatrix* PyramidMapMatrix::GetCountMatrixSafe(unsigned int level) {
  if (!has_count_) {
    std::cerr << "PyramidMapMatrix: [GetCountMatrixSafe] Has no count."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr
        << "PyramidMapMatrix: [GetCountMatrixSafe] The level id is illegal."
        << std::endl;
    return NULL;
  }

  return &count_matrixes_[level];
}

UIntMatrix* PyramidMapMatrix::GetGroundCountMatrixSafe(unsigned int level) {
  if (!has_ground_count_) {
    std::cerr
        << "PyramidMapMatrix: [GetGroundCountMatrixSafe] Has no ground_count."
        << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetGroundCountMatrixSafe] The level id is "
                 "illegal."
              << std::endl;
    return NULL;
  }

  return &ground_count_matrixes_[level];
}

const FloatMatrix* PyramidMapMatrix::GetIntensityMatrixSafe(
    unsigned int level) const {
  if (!has_intensity_) {
    std::cerr << "PyramidMapMatrix: [GetIntensityMatrixSafe] Has no intensity."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr
        << "PyramidMapMatrix: [GetIntensityMatrixSafe] The level id is illegal."
        << std::endl;
    return NULL;
  }

  return &intensity_matrixes_[level];
}

const FloatMatrix* PyramidMapMatrix::GetIntensityVarMatrixSafe(
    unsigned int level) const {
  if (!has_intensity_var_) {
    std::cerr
        << "PyramidMapMatrix: [GetIntensityVarMatrixSafe] Has no intensity_var."
        << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetIntensityVarMatrixSafe] The level id "
                 "is illegal."
              << std::endl;
    return NULL;
  }

  return &intensity_var_matrixes_[level];
}

const FloatMatrix* PyramidMapMatrix::GetAltitudeMatrixSafe(
    unsigned int level) const {
  if (!has_altitude_) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeMatrixSafe] Has no altitude."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr
        << "PyramidMapMatrix: [GetAltitudeMatrixSafe] The level id is illegal."
        << std::endl;
    return NULL;
  }

  return &altitude_matrixes_[level];
}

const FloatMatrix* PyramidMapMatrix::GetAltitudeVarMatrixSafe(
    unsigned int level) const {
  if (!has_altitude_var_) {
    std::cerr
        << "PyramidMapMatrix: [GetAltitudeVarMatrixSafe] Has no altitude_var."
        << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetAltitudeVarMatrixSafe] The level id is "
                 "illegal."
              << std::endl;
    return NULL;
  }

  return &altitude_var_matrixes_[level];
}

const FloatMatrix* PyramidMapMatrix::GetGroundAltitudeMatrixSafe(
    unsigned int level) const {
  if (!has_ground_altitude_) {
    std::cerr << "PyramidMapMatrix: [GetGroundAltitudeMatrixSafe] Has no "
                 "ground_altitude."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetGroundAltitudeMatrixSafe] The level id "
                 "is illegal."
              << std::endl;
    return NULL;
  }

  return &ground_altitude_matrixes_[level];
}

const UIntMatrix* PyramidMapMatrix::GetCountMatrixSafe(
    unsigned int level) const {
  if (!has_count_) {
    std::cerr << "PyramidMapMatrix: [GetCountMatrixSafe] Has no count."
              << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr
        << "PyramidMapMatrix: [GetCountMatrixSafe] The level id is illegal."
        << std::endl;
    return NULL;
  }

  return &count_matrixes_[level];
}

const UIntMatrix* PyramidMapMatrix::GetGroundCountMatrixSafe(
    unsigned int level) const {
  if (!has_ground_count_) {
    std::cerr
        << "PyramidMapMatrix: [GetGroundCountMatrixSafe] Has no ground_count."
        << std::endl;
    return NULL;
  }

  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [GetGroundCountMatrixSafe] The level id is "
                 "illegal."
              << std::endl;
    return NULL;
  }

  return &ground_count_matrixes_[level];
}

void PyramidMapMatrix::SetIntensityMatrix(const float* input, unsigned int size,
                                          unsigned int start_index,
                                          unsigned int level) {
  if (!has_intensity_) {
    std::cerr << "PyramidMapMatrix: [SetIntensityMatrix] Has no intensity."
              << std::endl;
    return;
  }

  if (!CheckLegalityForSetData(level, start_index, size)) {
    std::cerr << "PyramidMapMatrix: [SetIntensityMatrix] Params is illegal."
              << std::endl;
    return;
  }

  intensity_matrixes_[level].SetData(input, size, start_index);
}

void PyramidMapMatrix::SetIntensityVarMatrix(const float* input,
                                             unsigned int size,
                                             unsigned int start_index,
                                             unsigned int level) {
  if (!has_intensity_var_) {
    std::cerr
        << "PyramidMapMatrix: [set_intensity_var_matrix] Has no intensity_var."
        << std::endl;
    return;
  }

  if (!CheckLegalityForSetData(level, start_index, size)) {
    std::cerr
        << "PyramidMapMatrix: [set_intensity_var_matrix] Params is illegal."
        << std::endl;
    return;
  }

  intensity_var_matrixes_[level].SetData(input, size, start_index);
}

void PyramidMapMatrix::SetAltitudeMatrix(const float* input, unsigned int size,
                                         unsigned int start_index,
                                         unsigned int level) {
  if (!has_altitude_) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeMatrix] Has no altitude."
              << std::endl;
    return;
  }

  if (!CheckLegalityForSetData(level, start_index, size)) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeMatrix] Params is illegal."
              << std::endl;
    return;
  }

  altitude_matrixes_[level].SetData(input, size, start_index);
}

void PyramidMapMatrix::SetAltitudeVarMatrix(const float* input,
                                            unsigned int size,
                                            unsigned int start_index,
                                            unsigned int level) {
  if (!has_altitude_var_) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeVarMatrix] Has no altitude_var."
              << std::endl;
    return;
  }

  if (!CheckLegalityForSetData(level, start_index, size)) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeVarMatrix] Params is illegal."
              << std::endl;
    return;
  }

  altitude_var_matrixes_[level].SetData(input, size, start_index);
}

void PyramidMapMatrix::SetGroundAltitudeMatrix(const float* input,
                                               unsigned int size,
                                               unsigned int start_index,
                                               unsigned int level) {
  if (!has_ground_altitude_) {
    std::cerr
        << "PyramidMapMatrix: [SetGroundAltitudeMatrix] Has no ground_altitude."
        << std::endl;
    return;
  }

  if (!CheckLegalityForSetData(level, start_index, size)) {
    std::cerr
        << "PyramidMapMatrix: [SetGroundAltitudeMatrix] Params is illegal."
        << std::endl;
    return;
  }

  ground_altitude_matrixes_[level].SetData(input, size, start_index);
}

void PyramidMapMatrix::SetCountMatrix(const unsigned int* input,
                                      unsigned int size,
                                      unsigned int start_index,
                                      unsigned int level) {
  if (!has_count_) {
    std::cerr << "PyramidMapMatrix: [SetCountMatrix] Has no count."
              << std::endl;
    return;
  }

  if (!CheckLegalityForSetData(level, start_index, size)) {
    std::cerr << "PyramidMapMatrix: [SetCountMatrix] Params is illegal."
              << std::endl;
    return;
  }

  count_matrixes_[level].SetData(input, size, start_index);
}

void PyramidMapMatrix::SetGroundCountMatrix(const unsigned int* input,
                                            unsigned int size,
                                            unsigned int start_index,
                                            unsigned int level) {
  if (!has_ground_count_) {
    std::cerr << "PyramidMapMatrix: [SetGroundCountMatrix] Has no ground count."
              << std::endl;
    return;
  }

  if (!CheckLegalityForSetData(level, start_index, size)) {
    std::cerr << "PyramidMapMatrix: [SetGroundCountMatrix] Params is illegal."
              << std::endl;
    return;
  }

  ground_count_matrixes_[level].SetData(input, size, start_index);
}

void PyramidMapMatrix::SetFloatMatrixRoi(const FloatMatrix* source_matrix,
                                         const Rect2D<unsigned int>& source_roi,
                                         const Rect2D<unsigned int>& target_roi,
                                         unsigned int type,
                                         unsigned int level) {
  if (source_matrix == NULL) {
    std::cerr << "PyramidMapMatrix: [SetFloatMatrixRoi] Source matrix is null."
              << std::endl;
    return;
  }

  switch (type) {
    case 0:
      if (!has_intensity_) {
        std::cerr << "PyramidMapMatrix: [SetFloatMatrixRoi] Has no intensity."
                  << std::endl;
        return;
      }
      break;
    case 1:
      if (!has_intensity_var_) {
        std::cerr
            << "PyramidMapMatrix: [SetFloatMatrixRoi] Has no intensity var."
            << std::endl;
        return;
      }
      break;
    case 2:
      if (!has_altitude_) {
        std::cerr << "PyramidMapMatrix: [SetFloatMatrixRoi] Has no altitude."
                  << std::endl;
        return;
      }
      break;
    case 3:
      if (!has_altitude_var_) {
        std::cerr
            << "PyramidMapMatrix: [SetFloatMatrixRoi] Has no altitude var."
            << std::endl;
        return;
      }
      break;
    case 4:
      if (!has_ground_altitude_) {
        std::cerr
            << "PyramidMapMatrix: [SetFloatMatrixRoi] Has no ground altitude."
            << std::endl;
        return;
      }
      break;
  }

  if (!CheckLegalityForSetDataRoi(
          level, static_cast<unsigned int>(source_matrix->GetRow()),
          static_cast<unsigned int>(source_matrix->GetCol()), source_roi,
          target_roi)) {
    std::cerr << "PyramidMapMatrix: [SetFloatMatrixRoi] Params is illegal."
              << std::endl;
    return;
  }

  const unsigned int& source_roi_min_x = source_roi.GetMinX();
  const unsigned int& source_roi_min_y = source_roi.GetMinY();

  const unsigned int& target_roi_min_x = target_roi.GetMinX();
  const unsigned int& target_roi_min_y = target_roi.GetMinY();
  const unsigned int& target_roi_max_x = target_roi.GetMaxX();
  const unsigned int& target_roi_max_y = target_roi.GetMaxY();

  // unsigned int roi_rows = target_roi_max_y - target_roi_min_y + 1;
  unsigned int roi_cols = target_roi_max_x - target_roi_min_x + 1;

  unsigned int inc = 0;
  for (unsigned int y = target_roi_min_y; y <= target_roi_max_y; ++y) {
    unsigned int target_start_index = y * cols_mr_[level] + target_roi_min_x;
    unsigned int source_start_index =
        (source_roi_min_y + inc) * source_matrix->GetCol() + source_roi_min_x;
    const float* input = (*source_matrix)[0] + source_start_index;

    switch (type) {
      case 0:
        SetIntensityMatrix(input, roi_cols, target_start_index, level);
        break;
      case 1:
        SetIntensityVarMatrix(input, roi_cols, target_start_index, level);
        break;
      case 2:
        SetAltitudeMatrix(input, roi_cols, target_start_index, level);
        break;
      case 3:
        SetAltitudeVarMatrix(input, roi_cols, target_start_index, level);
        break;
      case 4:
        SetGroundAltitudeMatrix(input, roi_cols, target_start_index, level);
        break;
    }
    ++inc;
  }
}

void PyramidMapMatrix::SetUintMatrixRoi(const UIntMatrix* source_matrix,
                                        const Rect2D<unsigned int>& source_roi,
                                        const Rect2D<unsigned int>& target_roi,
                                        unsigned int type, unsigned int level) {
  if (source_matrix == NULL) {
    std::cerr << "PyramidMapMatrix: [SetUintMatrixRoi] Source matrix is null."
              << std::endl;
    return;
  }

  switch (type) {
    case 0:
      if (!has_count_) {
        std::cerr << "PyramidMapMatrix: [SetUintMatrixRoi] Has no count."
                  << std::endl;
        return;
      }
      break;
    case 1:
      if (!has_ground_count_) {
        std::cerr << "PyramidMapMatrix: [SetUintMatrixRoi] Has no ground count."
                  << std::endl;
        return;
      }
      break;
  }

  if (!CheckLegalityForSetDataRoi(
          level, static_cast<unsigned int>(source_matrix->GetRow()),
          static_cast<unsigned int>(source_matrix->GetCol()), source_roi,
          target_roi)) {
    std::cerr << "PyramidMapMatrix: [SetUintMatrixRoi] Params is illegal."
              << std::endl;
    return;
  }

  const unsigned int& source_roi_min_x = source_roi.GetMinX();
  const unsigned int& source_roi_min_y = source_roi.GetMinY();
  // const unsigned int &source_roi_max_x = source_roi.GetMaxX();
  // const unsigned int &source_roi_max_y = source_roi.GetMaxY();

  const unsigned int& target_roi_min_x = target_roi.GetMinX();
  const unsigned int& target_roi_min_y = target_roi.GetMinY();
  const unsigned int& target_roi_max_x = target_roi.GetMaxX();
  const unsigned int& target_roi_max_y = target_roi.GetMaxY();

  // unsigned int roi_rows = target_roi_max_y - target_roi_min_y + 1;
  unsigned int roi_cols = target_roi_max_x - target_roi_min_x + 1;

  unsigned int inc = 0;
  for (unsigned int y = target_roi_min_y; y <= target_roi_max_y; ++y) {
    unsigned int target_start_index = y * cols_mr_[level] + target_roi_min_x;
    unsigned int source_start_index =
        (source_roi_min_y + inc) * source_matrix->GetCol() + source_roi_min_x;
    const unsigned int* input = (*source_matrix)[0] + source_start_index;

    switch (type) {
      case 0:
        SetCountMatrix(input, roi_cols, target_start_index, level);
        break;
      case 1:
        SetGroundCountMatrix(input, roi_cols, target_start_index, level);
        break;
    }
    ++inc;
  }
}

void PyramidMapMatrix::SetIntensitySafe(float intensity, unsigned int row,
                                        unsigned int col, unsigned int level) {
  if (!has_intensity_) {
    std::cerr << "PyramidMapMatrix: [SetIntensitySafe] Has no intensity."
              << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetIntensitySafe] Params is illegal."
              << std::endl;
    return;
  }

  intensity_matrixes_[level][row][col] = intensity;
}

void PyramidMapMatrix::SetIntensityVarSafe(float intensity_var,
                                           unsigned int row, unsigned int col,
                                           unsigned int level) {
  if (!has_intensity_var_) {
    std::cerr << "PyramidMapMatrix: [SetIntensityVarSafe] Has no intensity_var."
              << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetIntensityVarSafe] Params is illegal."
              << std::endl;
    return;
  }

  intensity_var_matrixes_[level][row][col] = intensity_var;
}

void PyramidMapMatrix::SetAltitudeSafe(float altitude, unsigned int row,
                                       unsigned int col, unsigned int level) {
  if (!has_altitude_) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeSafe] Has no altitude."
              << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeSafe] Params is illegal."
              << std::endl;
    return;
  }

  altitude_matrixes_[level][row][col] = altitude;
}

void PyramidMapMatrix::SetAltitudeVarSafe(float altitude_var, unsigned int row,
                                          unsigned int col,
                                          unsigned int level) {
  if (!has_altitude_var_) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeVarSafe] Has no altitude var."
              << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetAltitudeVarSafe] Params is illegal."
              << std::endl;
    return;
  }

  altitude_var_matrixes_[level][row][col] = altitude_var;
}

void PyramidMapMatrix::SetGroundAltitudeSafe(float ground_altitude,
                                             unsigned int row, unsigned int col,
                                             unsigned int level) {
  if (!has_ground_altitude_) {
    std::cerr
        << "PyramidMapMatrix: [SetGroundAltitudeSafe] Has no ground altitude."
        << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetGroundAltitudeSafe] Params is illegal."
              << std::endl;
    return;
  }

  ground_altitude_matrixes_[level][row][col] = ground_altitude;
}

void PyramidMapMatrix::SetCountSafe(unsigned int count, unsigned int row,
                                    unsigned int col, unsigned int level) {
  if (!has_count_) {
    std::cerr << "PyramidMapMatrix: [SetCountSafe] Has no count." << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetCountSafe] Params is illegal."
              << std::endl;
    return;
  }

  count_matrixes_[level][row][col] = count;
}

void PyramidMapMatrix::SetGroundCountSafe(unsigned int ground_count,
                                          unsigned int row, unsigned int col,
                                          unsigned int level) {
  if (!has_ground_count_) {
    std::cerr << "PyramidMapMatrix: [SetGroundCountSafe] Has no ground count."
              << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetGroundCountSafe] Params is illegal."
              << std::endl;
    return;
  }

  ground_count_matrixes_[level][row][col] = ground_count;
}

void PyramidMapMatrix::SetValueSafe(unsigned char intensity, float altitude,
                                    unsigned int row, unsigned int col,
                                    unsigned int level) {
  if (!has_intensity_) {
    std::cerr << "PyramidMapMatrix: [SetValueSafe] Has no intensity."
              << std::endl;
    return;
  }

  if (!has_altitude_) {
    std::cerr << "PyramidMapMatrix: [SetValueSafe] Has no altitude."
              << std::endl;
    return;
  }

  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [SetValueSafe] Params is illegal."
              << std::endl;
    return;
  }

  intensity_matrixes_[level][row][col] = intensity;
  altitude_matrixes_[level][row][col] = altitude;
}

void PyramidMapMatrix::MergeCellSafe(
    const float* intensity, const float* intensity_var, const float* altitude,
    const float* altitude_var, const float* ground_altitude,
    const unsigned int* count, const unsigned int* ground_count,
    unsigned int row, unsigned int col, unsigned int level) {
  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [MergeCellSafe] Params is illegal."
              << std::endl;
    return;
  }

  if (count == NULL || !has_count_) {
    return;
  }

  unsigned int new_count = count_matrixes_[level][row][col] + *count;
  float p0 = static_cast<float>(count_matrixes_[level][row][col]) /
             static_cast<float>(new_count);
  float p1 = static_cast<float>(*count) / static_cast<float>(new_count);

  float intensity_diff = 0.0f;
  if (intensity != NULL && has_intensity_) {
    intensity_diff = intensity_matrixes_[level][row][col] - *intensity;
    intensity_matrixes_[level][row][col] =
        intensity_matrixes_[level][row][col] * p0 + *intensity * p1;
  }

  if (intensity != NULL && has_intensity_ && intensity_var != NULL &&
      has_intensity_var_) {
    intensity_var_matrixes_[level][row][col] =
        intensity_var_matrixes_[level][row][col] * p0 + *intensity_var * p1 +
        intensity_diff * intensity_diff * p0 * p1;
  }

  float altitude_diff = 0.0f;
  if (altitude != NULL && has_altitude_) {
    altitude_diff = altitude_matrixes_[level][row][col] - *altitude;
    altitude_matrixes_[level][row][col] =
        altitude_matrixes_[level][row][col] * p0 + *altitude * p1;
  }

  if (altitude != NULL && has_altitude_ && altitude_var != NULL &&
      has_altitude_var_) {
    altitude_var_matrixes_[level][row][col] =
        altitude_var_matrixes_[level][row][col] * p0 + *altitude_var * p1 +
        altitude_diff * altitude_diff * p0 * p1;
  }

  count_matrixes_[level][row][col] = new_count;

  // for points on ground
  if (ground_count == NULL || !has_ground_count_) {
    return;
  }

  unsigned int new_ground_count =
      ground_count_matrixes_[level][row][col] + *ground_count;
  p0 = static_cast<float>(ground_count_matrixes_[level][row][col]) /
       static_cast<float>(new_ground_count);
  p1 = static_cast<float>(*ground_count) /
       static_cast<float>(new_ground_count);

  if (ground_altitude != NULL && has_ground_altitude_) {
    ground_altitude_matrixes_[level][row][col] =
        ground_altitude_matrixes_[level][row][col] * p0 + *ground_altitude * p1;
  }

  ground_count_matrixes_[level][row][col] = new_ground_count;
}

bool PyramidMapMatrix::CheckLegalityForGetData(unsigned int row,
                                               unsigned int col,
                                               unsigned int level) const {
  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [CheckLegalityForGetData] The level id is "
                 "illegal."
              << std::endl;
    return false;
  }

  if (row >= rows_mr_[level]) {
    std::cerr
        << "PyramidMapMatrix: [CheckLegalityForGetData] The row is illegal."
        << std::endl;
    return false;
  }

  if (col >= cols_mr_[level]) {
    std::cerr
        << "PyramidMapMatrix: [CheckLegalityForGetData] The col is illegal."
        << std::endl;
    return false;
  }

  return true;
}

bool PyramidMapMatrix::CheckLegalityForSetData(unsigned int level,
                                               unsigned int start_id,
                                               unsigned int size) const {
  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [CheckLegalityForSetData] The level id is "
                 "illegal."
              << std::endl;
    return false;
  }

  if (start_id + size > rows_mr_[level] * cols_mr_[level]) {
    std::cerr << "PyramidMapMatrix: [CheckLegalityForSetData] The start_id or "
                 "size is illegal."
              << std::endl;
    return false;
  }

  return true;
}

bool PyramidMapMatrix::CheckLegalityForSetDataRoi(
    unsigned int level, unsigned int source_matrix_rows,
    unsigned int source_matrix_cols, const Rect2D<unsigned int>& source_roi,
    const Rect2D<unsigned int>& target_roi) const {
  if (level >= resolution_num_) {
    std::cerr << "PyramidMapMatrix: [CheckLegalityForSetDataRoi] The level id "
                 "is illegal."
              << std::endl;
    return false;
  }

  const unsigned int& source_roi_min_x = source_roi.GetMinX();
  const unsigned int& source_roi_min_y = source_roi.GetMinY();
  const unsigned int& source_roi_max_x = source_roi.GetMaxX();
  const unsigned int& source_roi_max_y = source_roi.GetMaxY();

  const unsigned int& target_roi_min_x = target_roi.GetMinX();
  const unsigned int& target_roi_min_y = target_roi.GetMinY();
  const unsigned int& target_roi_max_x = target_roi.GetMaxX();
  const unsigned int& target_roi_max_y = target_roi.GetMaxY();

  if (source_roi_min_x > source_roi_max_x ||
      source_roi_min_y > source_roi_max_y ||
      target_roi_min_x > target_roi_max_x ||
      target_roi_min_y > target_roi_max_y ||
      source_roi_max_x >= source_matrix_cols ||
      source_roi_max_y >= source_matrix_rows ||
      target_roi_max_x >= cols_mr_[level] ||
      target_roi_max_y >= rows_mr_[level] ||
      source_roi_max_x - source_roi_min_x !=
          target_roi_max_x - target_roi_min_x ||
      source_roi_max_y - source_roi_min_y !=
          target_roi_max_y - target_roi_min_y) {
    std::cerr << "PyramidMapMatrix: [CheckLegalityForSetDataRoi]"
                 " The source_roi or target_roi is illegal."
              << std::endl;
    return false;
  }

  return true;
}

void PyramidMapMatrix::AddSampleSafe(float intensity, float altitude,
                                     unsigned int row, unsigned int col,
                                     unsigned int level) {
  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [AddSampleSafe] Params is illegal."
              << std::endl;
    return;
  }

  if (has_count_) {
    ++count_matrixes_[level][row][col];
  }

  float v1 = 0.0;
  float v2 = 0.0;
  float value = 0.0;
  if (has_count_ && has_intensity_) {
    v1 = intensity - intensity_matrixes_[level][row][col];
    value = v1 / static_cast<float>(count_matrixes_[level][row][col]);
    intensity_matrixes_[level][row][col] += value;
  }

  if (has_count_ && has_intensity_ && has_intensity_var_) {
    v2 = intensity - intensity_matrixes_[level][row][col];
    intensity_var_matrixes_[level][row][col] =
        (static_cast<float>(count_matrixes_[level][row][col] - 1) *
             intensity_var_matrixes_[level][row][col] +
         v1 * v2) /
        static_cast<float>(count_matrixes_[level][row][col]);
  }

  if (has_count_ && has_altitude_) {
    v1 = altitude - altitude_matrixes_[level][row][col];
    value = v1 / static_cast<float>(count_matrixes_[level][row][col]);
    altitude_matrixes_[level][row][col] += value;
  }

  if (has_count_ && has_altitude_ && has_altitude_var_) {
    v2 = altitude - altitude_matrixes_[level][row][col];
    altitude_var_matrixes_[level][row][col] =
        (static_cast<float>(count_matrixes_[level][row][col] - 1) *
             altitude_var_matrixes_[level][row][col] +
         v1 * v2) /
        static_cast<float>(count_matrixes_[level][row][col]);
  }
}

void PyramidMapMatrix::AddGroundSample(float ground_altitude, unsigned int row,
                                       unsigned int col, unsigned int level) {
  if (!CheckLegalityForGetData(row, col, level)) {
    std::cerr << "PyramidMapMatrix: [AddGroundSample] Params is illegal."
              << std::endl;
    return;
  }

  if (has_ground_count_) {
    ++ground_count_matrixes_[level][row][col];
  }

  float v1 = 0.0;
  float value = 0.0;
  if (has_ground_count_ && has_ground_altitude_) {
    v1 = ground_altitude - ground_altitude_matrixes_[level][row][col];
    value = v1 / static_cast<float>(ground_count_matrixes_[level][row][col]);
    ground_altitude_matrixes_[level][row][col] += value;
  }
}

double PyramidMapMatrix::ComputeMeanIntensity(unsigned int level) {
  if (!has_count_) {
    std::cerr << "PyramidMapMatrix: [ComputeMeanIntensity] Has no count."
              << std::endl;
    return 0.0;
  }

  if (!has_intensity_ || resolution_num_ < 1) {
    std::cerr << "PyramidMapMatrix: [ComputeMeanIntensity] No intensity data."
              << std::endl;
    return 0.0;
  }

  if (level >= resolution_num_) {
    std::cerr
        << "PyramidMapMatrix: [ComputeMeanIntensity] The level id is illegal."
        << std::endl;
    return 0.0;
  }

  double avg = 0.0;
  int count = 0;
  for (unsigned int y = 0; y < rows_mr_[level]; ++y) {
    for (unsigned int x = 0; x < cols_mr_[level]; ++x) {
      if (count_matrixes_[level][y][x] > 0) {
        ++count;
        double delta = intensity_matrixes_[level][y][x] - avg;
        avg += (delta / count);
      }
    }
  }

  return avg;
}

void PyramidMapMatrix::Reduce(PyramidMapMatrix* cells,
                              const PyramidMapMatrix& new_cells,
                              unsigned int level, unsigned int new_level) {
  if (level >= cells->resolution_num_) {
    std::cerr << "PyramidMapMatrix: [Reduce] The level id is illegal."
              << std::endl;
    return;
  }

  if (new_level >= new_cells.resolution_num_) {
    std::cerr << "PyramidMapMatrix: [Reduce] The new level id is illegal."
              << std::endl;
    return;
  }

  if (cells->rows_mr_[level] != new_cells.rows_mr_[new_level] ||
      cells->cols_mr_[level] != new_cells.cols_mr_[new_level]) {
    return;
  }

  for (unsigned int r = 0; r < cells->rows_mr_[level]; r++) {
    for (unsigned int c = 0; c < cells->cols_mr_[level]; c++) {
      const float* intensity = new_cells.GetIntensitySafe(r, c, new_level);
      const float* intensity_var =
          new_cells.GetIntensityVarSafe(r, c, new_level);
      const float* altitude = new_cells.GetAltitudeSafe(r, c, new_level);
      const float* altitude_var = new_cells.GetAltitudeVarSafe(r, c, new_level);
      const float* ground_altitude =
          new_cells.GetGroundAltitudeSafe(r, c, new_level);
      const unsigned int* count = new_cells.GetCountSafe(r, c, new_level);
      const unsigned int* ground_count =
          new_cells.GetGroundCountSafe(r, c, new_level);

      cells->MergeCellSafe(intensity, intensity_var, altitude, altitude_var,
                           ground_altitude, count, ground_count, r, c, level);
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
