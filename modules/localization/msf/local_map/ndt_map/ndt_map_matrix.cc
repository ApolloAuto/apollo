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

#include "modules/localization/msf/local_map/ndt_map/ndt_map_matrix.h"
#include "modules/localization/msf/local_map/ndt_map/ndt_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

NdtMapSingleCell::NdtMapSingleCell() {
  intensity_ = 0.0;
  intensity_var_ = 0.0;
  road_pt_count_ = 0;
  count_ = 0;
  centroid_ = Eigen::Vector3f::Zero();
  centroid_average_cov_ = Eigen::Matrix3f::Zero();
  centroid_icov_ = Eigen::Matrix3f::Identity();
  is_icov_available_ = 0;
}

unsigned int NdtMapSingleCell::LoadBinary(unsigned char* buf) {
  float* f_buf = reinterpret_cast<float*>(buf);
  intensity_ = *f_buf;
  ++f_buf;
  intensity_var_ = *f_buf;
  ++f_buf;
  unsigned int* ui_buf =
      reinterpret_cast<unsigned int*>(reinterpret_cast<void*>(f_buf));
  road_pt_count_ = *ui_buf;
  ++ui_buf;
  count_ = *ui_buf;
  ++ui_buf;
  f_buf = reinterpret_cast<float*>(reinterpret_cast<void*>(ui_buf));
  centroid_[0] = *f_buf;
  ++f_buf;
  centroid_[1] = *f_buf;
  ++f_buf;
  centroid_[2] = *f_buf;

  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      ++f_buf;
      centroid_average_cov_(i, j) = *f_buf;
    }
  }
  if (count_ >= minimum_points_threshold_) {
    for (unsigned int i = 0; i < 3; ++i) {
      for (unsigned int j = 0; j < 3; ++j) {
        ++f_buf;
        centroid_icov_(i, j) = *f_buf;
      }
    }
    ++f_buf;
    unsigned char* uc_buf = reinterpret_cast<unsigned char*>(f_buf);
    is_icov_available_ = *uc_buf;
  } else {
    centroid_icov_ = Eigen::Matrix3f::Identity();
    is_icov_available_ = 0;
  }
  return GetBinarySize();
}

unsigned int NdtMapSingleCell::CreateBinary(unsigned char* buf,
                                            unsigned int buf_size) const {
  unsigned int target_size = GetBinarySize();
  if (buf_size >= target_size) {
    float* p = reinterpret_cast<float*>(buf);
    *p = intensity_;
    ++p;
    *p = intensity_var_;
    ++p;
    unsigned int* pp =
        reinterpret_cast<unsigned int*>(reinterpret_cast<void*>(p));
    *pp = road_pt_count_;
    ++pp;
    *pp = count_;
    ++pp;
    float* ppp = reinterpret_cast<float*>(reinterpret_cast<void*>(pp));
    *ppp = centroid_[0];
    ++ppp;
    *ppp = centroid_[1];
    ++ppp;
    *ppp = centroid_[2];

    for (unsigned int i = 0; i < 3; ++i) {
      for (unsigned int j = 0; j < 3; ++j) {
        ++ppp;
        *ppp = centroid_average_cov_(i, j);
      }
    }
    if (count_ >= minimum_points_threshold_) {
      for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
          ++ppp;
          *ppp = centroid_icov_(i, j);
        }
      }
      ++ppp;
      unsigned char* pppp = reinterpret_cast<unsigned char*>(ppp);
      *pppp = is_icov_available_;
    }
  }
  return target_size;
}

unsigned int NdtMapSingleCell::GetBinarySize() const {
  unsigned int sz =
      static_cast<unsigned int>(sizeof(float) * 2 + sizeof(unsigned int) * 2 +
                                sizeof(float) * 3 + sizeof(float) * 9);
  if (count_ >= minimum_points_threshold_) {
    sz += static_cast<unsigned int>(sizeof(float) * 9 +
                                    sizeof(unsigned char) * 1);
  }
  return sz;
}

NdtMapSingleCell& NdtMapSingleCell::operator=(const NdtMapSingleCell& ref) {
  count_ = ref.count_;
  intensity_ = ref.intensity_;
  intensity_var_ = ref.intensity_var_;
  road_pt_count_ = ref.road_pt_count_;
  centroid_ = ref.centroid_;
  centroid_average_cov_ = ref.centroid_average_cov_;
  centroid_icov_ = ref.centroid_icov_;
  is_icov_available_ = ref.is_icov_available_;
  return *this;
}

void NdtMapSingleCell::Reduce(NdtMapSingleCell* cell,
                              const NdtMapSingleCell& cell_new) {
  cell->MergeCell(cell_new);
}

NdtMapCells::NdtMapCells() {
  max_altitude_index_ = static_cast<int>(-1e10);
  min_altitude_index_ = static_cast<int>(1e10);
}

void NdtMapCells::Reset() {
  max_altitude_index_ = static_cast<int>(-1e10);
  min_altitude_index_ = static_cast<int>(1e10);
  cells_.clear();
  road_cell_indices_.clear();
}

int NdtMapCells::AddSample(const float intensity, const float altitude,
                           const float resolution,
                           const Eigen::Vector3f centroid, bool is_road) {
  int altitude_index = CalAltitudeIndex(resolution, altitude);
  NdtMapSingleCell& cell = cells_[altitude_index];
  cell.AddSample(intensity, altitude, centroid, is_road);
  if (altitude_index > max_altitude_index_) {
    max_altitude_index_ = altitude_index;
  }
  if (altitude_index < min_altitude_index_) {
    min_altitude_index_ = altitude_index;
  }
  if (is_road) {
    auto got = std::find(road_cell_indices_.begin(), road_cell_indices_.end(),
                         altitude_index);
    if (got == road_cell_indices_.end()) {
      road_cell_indices_.push_back(altitude_index);
    }
  }

  return altitude_index;
}

unsigned int NdtMapCells::LoadBinary(unsigned char* buf) {
  unsigned int* p = reinterpret_cast<unsigned int*>(buf);
  unsigned int size = *p;
  ++p;

  unsigned char* pp = reinterpret_cast<unsigned char*>(p);
  for (unsigned int i = 0; i < size; ++i) {
    int* ppp = reinterpret_cast<int*>(pp);
    int altitude_index = *ppp;
    ++ppp;
    pp = reinterpret_cast<unsigned char*>(ppp);
    NdtMapSingleCell cell;
    unsigned int processed_size = cell.LoadBinary(pp);
    cells_[altitude_index] = cell;
    pp += processed_size;
  }

  int* ppp = reinterpret_cast<int*>(pp);
  max_altitude_index_ = *ppp;
  ++ppp;
  min_altitude_index_ = *ppp;
  ++ppp;

  p = reinterpret_cast<unsigned int*>(ppp);
  size = *p;
  ++p;
  ppp = reinterpret_cast<int*>(p);
  for (unsigned int i = 0; i < size; ++i) {
    int index = *ppp;
    ++ppp;
    road_cell_indices_.push_back(index);
  }

  return GetBinarySize();
}

unsigned int NdtMapCells::CreateBinary(unsigned char* buf,
                                       unsigned int buf_size) const {
  unsigned int target_size = GetBinarySize();
  if (buf_size >= target_size) {
    unsigned int* p = reinterpret_cast<unsigned int*>(buf);
    *p = static_cast<unsigned int>(cells_.size());
    ++p;
    buf_size -= static_cast<unsigned int>(sizeof(unsigned int));
    unsigned char* pp = reinterpret_cast<unsigned char*>(p);
    for (auto it = cells_.begin(); it != cells_.end(); ++it) {
      int altitude_index = it->first;
      const NdtMapSingleCell& cell = it->second;
      int* ppp = reinterpret_cast<int*>(pp);
      *ppp = altitude_index;
      ++ppp;
      pp = reinterpret_cast<unsigned char*>(ppp);
      unsigned int processed_size = cell.CreateBinary(pp, buf_size);
      assert(buf_size >= processed_size);
      buf_size -= processed_size;
      pp += processed_size;
    }

    int* ppp = reinterpret_cast<int*>(pp);
    *ppp = max_altitude_index_;
    ++ppp;
    *ppp = min_altitude_index_;
    ++ppp;

    unsigned int size = static_cast<unsigned int>(road_cell_indices_.size());
    p = reinterpret_cast<unsigned int*>(ppp);
    *p = size;
    ++p;
    ppp = reinterpret_cast<int*>(p);
    for (unsigned int i = 0; i < size; ++i) {
      *ppp = road_cell_indices_[i];
      ++ppp;
    }
  }
  return target_size;
}

unsigned int NdtMapCells::GetBinarySize() const {
  unsigned int target_size = sizeof(unsigned int);
  for (auto it = cells_.begin(); it != cells_.end(); ++it) {
    target_size += static_cast<unsigned int>(sizeof(int));
    const NdtMapSingleCell& cell = it->second;
    target_size += cell.GetBinarySize();
  }

  target_size += static_cast<unsigned int>(sizeof(int) * 2);
  target_size += static_cast<unsigned int>(sizeof(unsigned int));
  target_size +=
      static_cast<unsigned int>(sizeof(int) * road_cell_indices_.size());

  return target_size;
}

int NdtMapCells::CalAltitudeIndex(const float resolution,
                                  const float altitude) {
  return static_cast<int>(altitude / resolution);
}

float NdtMapCells::CalAltitude(const float resolution,
                               const int altitude_index) {
  return static_cast<float>(resolution *
                            (static_cast<float>(altitude_index) + 0.5));
}

void NdtMapCells::Reduce(NdtMapCells* cell, const NdtMapCells& cell_new) {
  // Reduce cells
  for (auto it = cell_new.cells_.begin(); it != cell_new.cells_.end(); ++it) {
    int altitude_index = it->first;
    auto got = cell->cells_.find(altitude_index);
    if (got != cell->cells_.end()) {
      cell->cells_[altitude_index].MergeCell(it->second);
    } else {
      cell->cells_[altitude_index] = NdtMapSingleCell(it->second);
    }
  }

  if (cell_new.max_altitude_index_ > cell->max_altitude_index_) {
    cell->max_altitude_index_ = cell_new.max_altitude_index_;
  }

  if (cell_new.min_altitude_index_ < cell->min_altitude_index_) {
    cell->min_altitude_index_ = cell_new.min_altitude_index_;
  }

  for (auto it_new = cell_new.road_cell_indices_.begin();
       it_new != cell_new.road_cell_indices_.end(); ++it_new) {
    auto got_it = std::find(cell->road_cell_indices_.begin(),
                            cell->road_cell_indices_.end(), *it_new);
    if (got_it != cell->road_cell_indices_.end()) {
      *got_it += *it_new;
    } else {
      cell->road_cell_indices_.push_back(*it_new);
    }
  }
}

NdtMapMatrix::NdtMapMatrix() {
  rows_ = 0;
  cols_ = 0;
  map3d_cells_ = nullptr;
}

NdtMapMatrix::~NdtMapMatrix() {}

NdtMapMatrix::NdtMapMatrix(const NdtMapMatrix& cells) {
  Init(cells.rows_, cells.cols_);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      NdtMapCells& cell = GetMapCell(y, x);
      const NdtMapCells& src_cell = cells.GetMapCell(y, x);
      cell = NdtMapCells(src_cell);
    }
  }
}

void NdtMapMatrix::Init(const BaseMapConfig* config) {
  Init(config->map_node_size_y_, config->map_node_size_x_);
}

void NdtMapMatrix::Reset(const BaseMapConfig* config) {
  Reset(config->map_node_size_y_, config->map_node_size_x_);
}

void NdtMapMatrix::Init(unsigned int rows, unsigned int cols) {
  map3d_cells_.reset(new NdtMapCells[rows * cols]);
  rows_ = rows;
  cols_ = cols;
}

void NdtMapMatrix::Reset(unsigned int rows, unsigned int cols) {
  unsigned int length = rows * cols;
  for (unsigned int i = 0; i < length; ++i) {
    map3d_cells_[i].Reset();
  }
}

unsigned int NdtMapMatrix::LoadBinary(unsigned char* buf) {
  unsigned int* p = reinterpret_cast<unsigned int*>(buf);
  rows_ = *p;
  ++p;
  cols_ = *p;
  ++p;
  Init(rows_, cols_);
  unsigned char* pp = reinterpret_cast<unsigned char*>(p);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      NdtMapCells& cell = GetMapCell(y, x);
      unsigned int processed_size = cell.LoadBinary(pp);
      pp += processed_size;
    }
  }
  return GetBinarySize();
}

unsigned int NdtMapMatrix::CreateBinary(unsigned char* buf,
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
        const NdtMapCells& cell = GetMapCell(y, x);
        unsigned int processed_size = cell.CreateBinary(pp, buf_size);
        assert(buf_size >= processed_size);
        buf_size -= processed_size;
        pp += processed_size;
      }
    }
  }
  return target_size;
}

unsigned int NdtMapMatrix::GetBinarySize() const {
  unsigned int target_size =
      static_cast<unsigned int>(sizeof(unsigned int) * 2);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      const NdtMapCells& cell = GetMapCell(y, x);
      target_size += cell.GetBinarySize();
    }
  }
  return target_size;
}

void NdtMapMatrix::Reduce(NdtMapMatrix* cells, const NdtMapMatrix& cells_new) {
  for (unsigned int y = 0; y < cells->GetRows(); ++y) {
    for (unsigned int x = 0; x < cells->GetCols(); ++x) {
      NdtMapCells& cell = cells->GetMapCell(y, x);
      const NdtMapCells& cell_new = cells_new.GetMapCell(y, x);
      NdtMapCells::Reduce(&cell, cell_new);
    }
  }
}

void NdtMapMatrix::GetIntensityImg(cv::Mat* intensity_img) const {
  *intensity_img = cv::Mat(cv::Size(cols_, rows_), CV_8UC1);
  for (unsigned int y = 0; y < rows_; ++y) {
    for (unsigned int x = 0; x < cols_; ++x) {
      unsigned int id = y * cols_ + x;
      int min_altitude_index = map3d_cells_[id].min_altitude_index_;
      intensity_img->at<unsigned char>(y, x) =
          (unsigned char)(map3d_cells_[id]
                              .cells_[min_altitude_index]
                              .intensity_);
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
