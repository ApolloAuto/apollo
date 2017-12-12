/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"

namespace apollo {
namespace localization {
namespace msf {

LossyMapCell2D::LossyMapCell2D()
    : count(0),
      intensity(0.0),
      intensity_var(0.0),
      altitude(0.0),
      altitude_ground(0.0),
      is_ground_useful(false) {}

void LossyMapCell2D::Reset() {
  intensity = 0.0;
  intensity_var = 0.0;
  altitude = 0.0;
  altitude_ground = 0.0;
  is_ground_useful = false;
  count = 0;
}

LossyMapCell2D& LossyMapCell2D::operator=(const LossyMapCell2D& ref) {
  altitude = ref.altitude;
  altitude_ground = ref.altitude_ground;
  is_ground_useful = ref.is_ground_useful;
  count = ref.count;
  intensity = ref.intensity;
  intensity_var = ref.intensity_var;
  return *this;
}

LossyMapMatrix2D::LossyMapMatrix2D() {
  rows_ = 0;
  cols_ = 0;
  map_cells_ = NULL;
}

LossyMapMatrix2D::~LossyMapMatrix2D() {
  if (map_cells_) {
    delete[] map_cells_;
  }
  rows_ = 0;
  cols_ = 0;
}

LossyMapMatrix2D::LossyMapMatrix2D(const LossyMapMatrix2D& matrix)
    : BaseMapMatrix(matrix), map_cells_(NULL) {
  Init(matrix.rows_, matrix.cols_);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      map_cells_[y * cols_ + x] = matrix[y][x];
    }
  }
}

LossyMapMatrix2D& LossyMapMatrix2D::operator=(const LossyMapMatrix2D& matrix) {
  Init(matrix.rows_, matrix.cols_);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      map_cells_[y * cols_ + x] = matrix[y][x];
    }
  }
  return *this;
}

void LossyMapMatrix2D::Init(const BaseMapConfig* config) {
  unsigned int rows = config->map_node_size_y_;
  unsigned int cols = config->map_node_size_x_;
  if (rows_ == rows && cols_ == cols) {
    return;
  }
  Init(rows, cols);
  return;
}

void LossyMapMatrix2D::Init(unsigned int rows, unsigned int cols) {
  if (map_cells_) {
    delete[] map_cells_;
    map_cells_ = NULL;
  }
  map_cells_ = new LossyMapCell2D[rows * cols];
  rows_ = rows;
  cols_ = cols;
}

void LossyMapMatrix2D::Reset(const BaseMapConfig* config) {
  Reset(config->map_node_size_y_, config->map_node_size_x_);
  return;
}

void LossyMapMatrix2D::Reset(unsigned int rows, unsigned int cols) {
  unsigned int length = rows * cols;
  for (unsigned int i = 0; i < length; ++i) {
    map_cells_[i].Reset();
  }
}

unsigned char LossyMapMatrix2D::EncodeIntensity(
    const LossyMapCell2D& cell) const {
  int intensity = cell.intensity;
  if (intensity > 255) {
    intensity = 255;
  }
  if (intensity < 0) {
    intensity = 0;
  }
  return intensity;
}

void LossyMapMatrix2D::DecodeIntensity(unsigned char data,
                                       LossyMapCell2D* cell) const {
  cell->intensity = data;
}

uint16_t LossyMapMatrix2D::EncodeVar(const LossyMapCell2D& cell) const {
  float var = cell.intensity_var;
  var = std::sqrt(var);
  int intensity_var = var_range_ / (var * var_ratio_ + 1.0);
  if (intensity_var > var_range_) {
    intensity_var = var_range_;
  }
  if (intensity_var < 1) {
    intensity_var = 1;
  }
  return intensity_var;
}

void LossyMapMatrix2D::DecodeVar(uint16_t data, LossyMapCell2D* cell) const {
  float var = data;
  var = (var_range_ / var - 1.0) / var_ratio_;
  cell->intensity_var = var * var;
}

uint16_t LossyMapMatrix2D::EncodeAltitudeGround(
    const LossyMapCell2D& cell) const {
  float delta_alt = cell.altitude_ground - alt_ground_min_;
  delta_alt /= alt_ground_interval_;
  int ratio = delta_alt + 0.5;
  if (ratio >= ground_void_flag_) {
    ratio = ground_void_flag_ - 1;
  }
  if (ratio < 0) {
    ratio = 0;
  }
  return ratio;
}

void LossyMapMatrix2D::DecodeAltitudeGround(uint16_t data,
                                            LossyMapCell2D* cell) const {
  float ratio = data;
  cell->altitude_ground = alt_ground_min_ + ratio * alt_ground_interval_;
  return;
}

uint16_t LossyMapMatrix2D::EncodeAltitudeAvg(const LossyMapCell2D& cell) const {
  float delta_alt = cell.altitude - alt_avg_min_;
  delta_alt /= alt_avg_interval_;
  int ratio = delta_alt + 0.5;
  if (ratio > 0xffff) {
    ratio = 0xffff;
  }
  if (ratio < 0) {
    ratio = 0;
  }
  return ratio;
}

void LossyMapMatrix2D::DecodeAltitudeAvg(uint16_t data,
                                         LossyMapCell2D* cell) const {
  float ratio = data;
  cell->altitude = alt_avg_min_ + ratio * alt_avg_interval_;
  return;
}

unsigned char LossyMapMatrix2D::EncodeCount(const LossyMapCell2D& cell) const {
  int count_exp = 0;
  int count_tmp = cell.count;
  while (count_tmp > 0) {
    ++count_exp;
    count_tmp /= 2;
  }
  if (count_exp > count_range_) {
    count_exp = count_range_;
  }
  return count_exp;
}

void LossyMapMatrix2D::DecodeCount(unsigned char data,
                                   LossyMapCell2D* cell) const {
  int count_exp = data;
  if (count_exp == 0) {
    cell->count = count_exp;
  } else {
    cell->count = 1 << (count_exp - 1);
  }
}

unsigned int LossyMapMatrix2D::LoadBinary(unsigned char* buf) {
  unsigned int* p = reinterpret_cast<unsigned int*>(buf);
  rows_ = *p;
  ++p;
  cols_ = *p;
  ++p;
  // std::cerr << "rows: " << rows_ << ", clos: " << cols_ << std::endl;
  float* pf = reinterpret_cast<float*>(p);
  alt_avg_min_ = *pf;
  ++pf;
  alt_avg_max_ = *pf;
  ++pf;
  // std::cerr << "alt_min: " << alt_avg_min_ << ", alt_max: " << alt_avg_max_
  // << std::endl;
  alt_ground_min_ = *pf;
  ++pf;
  alt_ground_max_ = *pf;
  ++pf;
  // std::cerr << "alt_min: " << _alt_min << ", alt_max: " << _alt_max <<
  // std::endl;

  Init(rows_, cols_);

  unsigned char* pp = reinterpret_cast<unsigned char*>(pf);
  // count
  for (unsigned int row = 0; row < rows_; ++row) {
    for (unsigned int col = 0; col < cols_; ++col) {
      DecodeCount(pp[row * cols_ + col], &map_cells_[row * cols_ + col]);
    }
  }
  pp += rows_ * cols_;

  // intensity
  for (unsigned int row = 0; row < rows_; ++row) {
    for (unsigned int col = 0; col < cols_; ++col) {
      DecodeIntensity(pp[row * cols_ + col], &map_cells_[row * cols_ + col]);
    }
  }
  pp += rows_ * cols_;

  // intensity_var
  unsigned char* pp_low = pp + rows_ * cols_;
  unsigned char* pp_high = pp;
  for (unsigned int row = 0; row < rows_; ++row) {
    for (unsigned int col = 0; col < cols_; ++col) {
      uint16_t var = pp_high[row * cols_ + col];
      var = var * 256 + pp_low[row * cols_ + col];
      DecodeVar(var, &map_cells_[row * cols_ + col]);
    }
  }
  pp += 2 * rows_ * cols_;

  // altitude_avg
  pp_low = pp + rows_ * cols_;
  pp_high = pp;
  for (unsigned int row = 0; row < rows_; ++row) {
    for (unsigned int col = 0; col < cols_; ++col) {
      uint16_t alt = pp_high[row * cols_ + col];
      alt = alt * 256 + pp_low[row * cols_ + col];
      LossyMapCell2D& cell = map_cells_[row * cols_ + col];
      if (cell.count > 0) {
        DecodeAltitudeAvg(alt, &cell);
      } else {
        cell.altitude = 0.0;
      }
    }
  }
  pp += 2 * rows_ * cols_;

  // altitude_ground
  pp_low = pp + rows_ * cols_;
  pp_high = pp;
  for (unsigned int row = 0; row < rows_; ++row) {
    for (unsigned int col = 0; col < cols_; ++col) {
      uint16_t alt = pp_high[row * cols_ + col];
      alt = alt * 256 + pp_low[row * cols_ + col];
      LossyMapCell2D& cell = map_cells_[row * cols_ + col];
      if (alt == ground_void_flag_) {
        cell.is_ground_useful = false;
        cell.altitude_ground = 0.0;
      } else {
        cell.is_ground_useful = true;
        DecodeAltitudeGround(alt, &cell);
      }
    }
  }
  // TODO(Localization): remove this line
  // pp += 2 * rows_ * cols_;

  return GetBinarySize();
}

unsigned int LossyMapMatrix2D::CreateBinary(unsigned char* buf,
                                            unsigned int buf_size) const {
  unsigned int target_size = GetBinarySize();
  if (buf_size >= target_size) {
    unsigned int* p = reinterpret_cast<unsigned int*>(buf);
    *p = rows_;
    ++p;
    *p = cols_;
    ++p;
    buf_size -= sizeof(unsigned int) * 2;

    float* pf = reinterpret_cast<float*>(p);
    alt_avg_min_ = 1e8;
    alt_avg_max_ = -1e8;
    for (unsigned int y = 0; y < rows_; ++y) {
      for (unsigned int x = 0; x < cols_; ++x) {
        const LossyMapCell2D& cell = map_cells_[y * cols_ + x];
        if (cell.count == 0) {
          continue;
        }
        if (cell.altitude > alt_avg_max_) {
          alt_avg_max_ = cell.altitude;
        }
        if (cell.altitude < alt_avg_min_) {
          alt_avg_min_ = cell.altitude;
        }
      }
    }
    *pf = alt_avg_min_;
    ++pf;
    *pf = alt_avg_max_;
    ++pf;
    buf_size -= sizeof(float) * 2;

    alt_ground_min_ = 1e8;
    alt_ground_max_ = -1e8;
    for (unsigned int y = 0; y < rows_; ++y) {
      for (unsigned int x = 0; x < cols_; ++x) {
        const LossyMapCell2D& cell = map_cells_[y * cols_ + x];
        if (cell.is_ground_useful == false) {
          continue;
        }
        if (cell.altitude_ground > alt_ground_max_) {
          alt_ground_max_ = cell.altitude_ground;
        }
        if (cell.altitude_ground < alt_ground_min_) {
          alt_ground_min_ = cell.altitude_ground;
        }
      }
    }
    *pf = alt_ground_min_;
    ++pf;
    *pf = alt_ground_max_;
    ++pf;
    buf_size -= sizeof(float) * 2;

    unsigned char* pp = reinterpret_cast<unsigned char*>(pf);
    // count
    for (unsigned int row = 0; row < rows_; ++row) {
      for (unsigned int col = 0; col < cols_; ++col) {
        pp[row * cols_ + col] = EncodeCount(map_cells_[row * cols_ + col]);
      }
    }
    pp += rows_ * cols_;

    // intensity
    for (unsigned int row = 0; row < rows_; ++row) {
      for (unsigned int col = 0; col < cols_; ++col) {
        pp[row * cols_ + col] = EncodeIntensity(map_cells_[row * cols_ + col]);
      }
    }
    pp += rows_ * cols_;

    // intensity_var
    unsigned char* pp_low = pp + rows_ * cols_;
    unsigned char* pp_high = pp;
    for (unsigned int row = 0; row < rows_; ++row) {
      for (unsigned int col = 0; col < cols_; ++col) {
        uint16_t var = EncodeVar(map_cells_[row * cols_ + col]);
        pp_high[row * cols_ + col] = var / 256;
        pp_low[row * cols_ + col] = var % 256;
      }
    }
    pp += 2 * rows_ * cols_;

    // altitude_avg
    pp_low = pp + rows_ * cols_;
    pp_high = pp;
    for (unsigned int row = 0; row < rows_; ++row) {
      for (unsigned int col = 0; col < cols_; ++col) {
        uint16_t altitude = 0.0;
        if (map_cells_[row * cols_ + col].count > 0) {
          altitude = EncodeAltitudeAvg(map_cells_[row * cols_ + col]);
        }
        pp_high[row * cols_ + col] = altitude / 256;
        pp_low[row * cols_ + col] = altitude % 256;
      }
    }
    pp += 2 * rows_ * cols_;

    // altitude_ground
    pp_low = pp + rows_ * cols_;
    pp_high = pp;
    for (unsigned int row = 0; row < rows_; ++row) {
      for (unsigned int col = 0; col < cols_; ++col) {
        uint16_t altitude = ground_void_flag_;
        if (map_cells_[row * cols_ + col].is_ground_useful) {
          altitude = EncodeAltitudeGround(map_cells_[row * cols_ + col]);
        }
        pp_high[row * cols_ + col] = altitude / 256;
        pp_low[row * cols_ + col] = altitude % 256;
      }
    }
    // TODO(Localization): remove this line
    // pp += 2 * rows_ * cols_;
  }
  return target_size;
}

unsigned int LossyMapMatrix2D::GetBinarySize() const {
  unsigned int target_size =
      sizeof(unsigned int) * 2 + sizeof(float) * 4;  // rows and cols and alts
  // count, intensity, intensity_var, altitude_avg, altitude_ground
  target_size +=
      rows_ * cols_ * (sizeof(unsigned char) + sizeof(unsigned char) +
                       sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t));
  return target_size;
}

void LossyMapMatrix2D::GetIntensityImg(cv::Mat* intensity_img) const {
  *intensity_img = cv::Mat(cv::Size(cols_, rows_), CV_8UC1);

  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      unsigned int id = y * cols_ + x;
      intensity_img->at<unsigned char>(y, x) =
          (unsigned char)(map_cells_[id].intensity);
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
