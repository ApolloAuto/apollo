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

#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

// ======================LosslessMapSingleCell===========================
LosslessMapSingleCell::LosslessMapSingleCell()
    : intensity(0.0),
      intensity_var(0.0),
      altitude(0.0),
      altitude_var(0.0),
      count(0) {}

void LosslessMapSingleCell::Reset() {
  intensity = 0.0;
  intensity_var = 0.0;
  altitude = 0.0;
  count = 0;
}

LosslessMapSingleCell& LosslessMapSingleCell::operator=(
    const LosslessMapSingleCell& ref) {
  intensity = ref.intensity;
  intensity_var = ref.intensity_var;
  altitude = ref.altitude;
  count = ref.count;
  return *this;
}

void LosslessMapSingleCell::AddSample(const float new_altitude,
                                      const float new_intensity) {
  ++count;
  float fcount = static_cast<float>(count);
  float v1 = new_intensity - intensity;
  float value = v1 / fcount;
  intensity += value;
  float v2 = new_intensity - intensity;
  intensity_var = (fcount - 1.0f) * intensity_var + v1 * v2 / fcount;

  v1 = new_altitude - altitude;
  value = v1 / fcount;
  altitude += value;
  v2 = new_altitude - altitude;
  altitude_var = ((fcount - 1) * altitude_var + v1 * v2) / fcount;
}

unsigned int LosslessMapSingleCell::LoadBinary(unsigned char* buf) {
  float* p = reinterpret_cast<float*>(buf);
  intensity = *p;
  ++p;
  intensity_var = *p;
  ++p;
  altitude = *p;
  ++p;
  altitude_var = *p;
  ++p;
  unsigned int* pp =
      reinterpret_cast<unsigned int*>(reinterpret_cast<void*>(p));
  count = *pp;
  return GetBinarySize();
}

unsigned int LosslessMapSingleCell::CreateBinary(unsigned char* buf,
                                                 unsigned int buf_size) const {
  unsigned int target_size = GetBinarySize();
  if (buf_size >= target_size) {
    float* p = reinterpret_cast<float*>(buf);
    *p = intensity;
    ++p;
    *p = intensity_var;
    ++p;
    *p = altitude;
    ++p;
    *p = altitude_var;
    ++p;
    unsigned int* pp =
        reinterpret_cast<unsigned int*>(reinterpret_cast<void*>(p));
    *pp = count;
  }
  return target_size;
}

unsigned int LosslessMapSingleCell::GetBinarySize() const {
  return static_cast<unsigned int>(sizeof(float) * 4 + sizeof(unsigned int));
}

// ======================LosslessMapCell===========================
LosslessMapCell::LosslessMapCell() { layer_num = 1; }

void LosslessMapCell::Reset() {
  for (unsigned int i = 0; i < IDL_CAR_NUM_RESERVED_MAP_LAYER; ++i) {
    map_cells[i].Reset();
  }
  layer_num = 1;
}

void LosslessMapCell::SetValueLayer(double altitude, unsigned char intensity,
                                    double altitude_thres) {
  DCHECK_LE(layer_num, IDL_CAR_NUM_RESERVED_MAP_LAYER);

  unsigned int best_layer_id = GetLayerId(altitude);
  DCHECK_LT(best_layer_id, layer_num);
  if (best_layer_id == 0) {
    if (layer_num < IDL_CAR_NUM_RESERVED_MAP_LAYER) {
      // No layer yet, create a new one
      LosslessMapSingleCell& cell = map_cells[layer_num++];
      cell.AddSample(static_cast<float>(altitude),
                     static_cast<float>(intensity));
    } else {
      // No enough reserved map layers.
      std::cerr << "[Warning] There are no enough reserved map cell layers. "
                   "Please increase IDL_CAR_NUM_RESERVED_MAP_LAYER."
                << std::endl;
    }
  } else {
    // There is a best layer
    double layer_alt_dif = fabs(map_cells[best_layer_id].altitude - altitude);
    if (layer_alt_dif < altitude_thres) {
      // Still a good one
      LosslessMapSingleCell& cell = map_cells[best_layer_id];
      cell.AddSample(static_cast<float>(altitude),
                     static_cast<float>(intensity));
    } else {
      // Should create a new one
      if (layer_num < IDL_CAR_NUM_RESERVED_MAP_LAYER) {
        LosslessMapSingleCell& cell = map_cells[layer_num++];
        cell.AddSample(static_cast<float>(altitude),
                       static_cast<float>(intensity));
      } else {
        // No enough reserved map layers.
        std::cerr << "[Warning] There are no enough reserved map cell layers. "
                     "Please increase IDL_CAR_NUM_RESERVED_MAP_LAYER."
                  << std::endl;
      }
    }
  }
}

void LosslessMapCell::SetValue(double altitude, unsigned char intensity) {
  DCHECK_LE(layer_num, IDL_CAR_NUM_RESERVED_MAP_LAYER);
  LosslessMapSingleCell& cell = map_cells[0];
  cell.AddSample(static_cast<float>(altitude), static_cast<float>(intensity));
}

unsigned int LosslessMapCell::LoadBinary(unsigned char* buf) {
  unsigned int* p = reinterpret_cast<unsigned int*>(buf);
  unsigned int size = *p;
  ++p;
  layer_num = size;
  unsigned char* pp = reinterpret_cast<unsigned char*>(p);
  for (unsigned int i = 0; i < size; ++i) {
    LosslessMapSingleCell& cell = map_cells[i];
    unsigned int processed_size = cell.LoadBinary(pp);
    pp += processed_size;
  }
  return GetBinarySize();
}

unsigned int LosslessMapCell::CreateBinary(unsigned char* buf,
                                           unsigned int buf_size) const {
  unsigned int target_size = GetBinarySize();
  if (buf_size >= target_size) {
    unsigned int* p = reinterpret_cast<unsigned int*>(buf);
    *p = layer_num;
    ++p;
    buf_size -= static_cast<unsigned int>(sizeof(unsigned int));
    unsigned char* pp = reinterpret_cast<unsigned char*>(p);
    for (size_t i = 0; i < layer_num; ++i) {
      const LosslessMapSingleCell& cell = map_cells[i];
      unsigned int processed_size = cell.CreateBinary(pp, buf_size);
      DCHECK_GE(buf_size, processed_size);
      buf_size -= processed_size;
      pp += processed_size;
    }
  }
  return target_size;
}

unsigned int LosslessMapCell::GetBinarySize() const {
  unsigned int target_size = sizeof(
      unsigned int);  // The size of the variable for the number of layers.
  for (size_t i = 0; i < layer_num; ++i) {
    const LosslessMapSingleCell& cell = map_cells[i];
    target_size += cell.GetBinarySize();
  }
  return target_size;
}

unsigned int LosslessMapCell::GetLayerId(double altitude) const {
  unsigned int best_layer_id = 0;
  double best_layer_alt_dif = 1e10;
  for (unsigned int i = 1; i < layer_num; ++i) {
    const LosslessMapSingleCell& cell = map_cells[i];
    double layer_alt_dif = fabs(cell.altitude - altitude);
    if (layer_alt_dif < best_layer_alt_dif) {
      best_layer_alt_dif = layer_alt_dif;
      best_layer_id = i;
    }
  }
  return best_layer_id;
}

void LosslessMapCell::GetValue(std::vector<unsigned char>* values) const {
  values->clear();
  for (unsigned int i = 1; i < layer_num; ++i) {
    const LosslessMapSingleCell& cell = map_cells[i];
    values->push_back(static_cast<unsigned char>(cell.intensity));
  }
}

void LosslessMapCell::GetVar(std::vector<float>* vars) const {
  vars->clear();
  for (unsigned int i = 1; i < layer_num; ++i) {
    const LosslessMapSingleCell& cell = map_cells[i];
    vars->push_back(cell.intensity_var);
  }
}

void LosslessMapCell::GetAlt(std::vector<float>* alts) const {
  alts->clear();
  for (unsigned int i = 1; i < layer_num; ++i) {
    const LosslessMapSingleCell& cell = map_cells[i];
    alts->push_back(cell.altitude);
  }
}

void LosslessMapCell::GetAltVar(std::vector<float>* alt_vars) const {
  alt_vars->clear();
  for (unsigned int i = 1; i < layer_num; ++i) {
    const LosslessMapSingleCell& cell = map_cells[i];
    alt_vars->push_back(cell.altitude_var);
  }
}

void LosslessMapCell::GetCount(std::vector<unsigned int>* counts) const {
  counts->clear();
  for (unsigned int i = 1; i < layer_num; ++i) {
    const LosslessMapSingleCell& cell = map_cells[i];
    counts->push_back(cell.count);
  }
}

// ======================LosslessMapMatrix===========================
LosslessMapMatrix::LosslessMapMatrix() {
  rows_ = 0;
  cols_ = 0;
  map_cells_ = nullptr;
}

LosslessMapMatrix::~LosslessMapMatrix() {
  if (map_cells_) {
    delete[] map_cells_;
  }
  rows_ = 0;
  cols_ = 0;
}

LosslessMapMatrix::LosslessMapMatrix(const LosslessMapMatrix& matrix)
    : BaseMapMatrix(matrix) {
  Init(matrix.rows_, matrix.cols_);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      map_cells_[y * cols_ + x] = matrix[y][x];
    }
  }
}

void LosslessMapMatrix::Init(const BaseMapConfig* config) {
  unsigned int rows = config->map_node_size_y_;
  unsigned int cols = config->map_node_size_x_;
  if (rows_ == rows && cols_ == cols) {
    return;
  }
  Init(rows, cols);
}

void LosslessMapMatrix::Reset(const BaseMapConfig* config) {
  Reset(config->map_node_size_y_, config->map_node_size_x_);
}

void LosslessMapMatrix::Init(unsigned int rows, unsigned int cols) {
  if (map_cells_) {
    delete[] map_cells_;
    map_cells_ = nullptr;
  }
  map_cells_ = new LosslessMapCell[rows * cols];
  rows_ = rows;
  cols_ = cols;
}

void LosslessMapMatrix::Reset(unsigned int rows, unsigned int cols) {
  unsigned int length = rows * cols;
  for (unsigned int i = 0; i < length; ++i) {
    map_cells_[i].Reset();
  }
}

unsigned int LosslessMapMatrix::LoadBinary(unsigned char* buf) {
  unsigned int* p = reinterpret_cast<unsigned int*>(buf);
  rows_ = *p;
  ++p;
  cols_ = *p;
  ++p;
  Init(rows_, cols_);

  unsigned char* pp = reinterpret_cast<unsigned char*>(p);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      LosslessMapCell& cell = GetMapCell(y, x);
      unsigned int processed_size = cell.LoadBinary(pp);
      pp += processed_size;
    }
  }
  return GetBinarySize();
}

unsigned int LosslessMapMatrix::CreateBinary(unsigned char* buf,
                                             unsigned int buf_size) const {
  unsigned int target_size = GetBinarySize();
  if (buf_size >= target_size) {
    unsigned int* p = reinterpret_cast<unsigned int*>(buf);
    *p = rows_;
    ++p;
    *p = cols_;
    ++p;
    buf_size -= static_cast<unsigned int>(sizeof(unsigned int) * 2);
    unsigned char* pp = reinterpret_cast<unsigned char*>(p);
    for (unsigned int y = 0; y < rows_; ++y) {
      for (unsigned int x = 0; x < cols_; ++x) {
        const LosslessMapCell& cell = GetMapCell(y, x);
        unsigned int processed_size = cell.CreateBinary(pp, buf_size);
        DCHECK_GE(buf_size, processed_size);
        buf_size -= processed_size;
        pp += processed_size;
      }
    }
  }
  return target_size;
}

unsigned int LosslessMapMatrix::GetBinarySize() const {
  // default binary size
  unsigned int target_size =
      static_cast<unsigned int>(sizeof(unsigned int) * 2);  // rows and cols
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      const LosslessMapCell& cell = GetMapCell(y, x);
      target_size += cell.GetBinarySize();
    }
  }
  return target_size;
}

void LosslessMapMatrix::GetIntensityImg(cv::Mat* intensity_img) const {
  *intensity_img = cv::Mat(cv::Size(cols_, rows_), CV_8UC1);

  for (uint32_t y = 0; y < rows_; ++y) {
    for (uint32_t x = 0; x < cols_; ++x) {
      intensity_img->at<unsigned char>(y, x) = GetMapCell(y, x).GetValue();
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
