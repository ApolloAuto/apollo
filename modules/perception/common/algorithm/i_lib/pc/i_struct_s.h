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

#include <vector>

#if defined(__x86_64__)
#include <smmintrin.h>
#elif defined(__aarch64__) && defined(__ARM_NEON)
#include "sse2neon.h" // NOLINT
#else
#error "Processor architecture not supported!"
#endif

#include "modules/perception/common/algorithm/i_lib/core/i_alloc.h"
#include "modules/perception/common/algorithm/i_lib/core/i_blas.h"
#include "modules/perception/common/algorithm/i_lib/pc/i_util.h"

namespace apollo {
namespace perception {
namespace algorithm {
template <typename T, unsigned int d>
class PtCluster {
 public:
  typedef T *iterator;
  typedef const T *const_iterator;
  typedef T &reference;
  typedef const T &const_reference;

  static unsigned int NrPointElement() { return (d); }

  static unsigned int PointsizeInByte() { return (d * sizeof(T)); }

  unsigned int ClustersizeInByte() const {
    return (d * sizeof(T) * nr_points_);
  }

  unsigned int NrPoints() const { return (nr_points_); }

  bool Initialized() const { return (nr_points_ > 0); }

  const T *ConstData() const { return (data_); }

  T *Data() { return (data_); }

  void CleanUp();

  PtCluster();
  explicit PtCluster(unsigned int n);
  PtCluster(const T *data, unsigned int n);
  PtCluster(const PtCluster<T, d> &c) : data_(nullptr), nr_points_(0) {
    if (c.NrPoints()) {
      nr_points_ = c.NrPoints();
      data_ = IAllocAligned<T>(nr_points_ * d, 4);
      assert(data_ != nullptr && IVerifyAlignment(data_, 4));
      ICopy(c.ConstData(), data_, nr_points_ * d);
    }
  }

  PtCluster &operator=(const PtCluster<T, d> &c) {
    if (this != &c) {
      if (this->nr_points_ != c.NrPoints()) {
        IFreeAligned(&data_);
        data_ = IAllocAligned<T>(this->nr_points_ * d, 4);
        assert(data_ != nullptr && IVerifyAlignment(data_, 4));
        this->nr_points_ = c.NrPoints();
      }
      ICopy(c.ConstData(), data_, this->nr_points_ * d);
    }
    return (*this);
  }

  virtual ~PtCluster() { CleanUp(); }

  T *operator[](unsigned int i) {
    assert(i < nr_points_);
    assert(data_ != nullptr);
    return (data_ + i * d);
  }

  const T *operator[](unsigned int i) const {
    assert(i < nr_points_);
    assert(data_ != nullptr);
    return (data_ + i * d);
  }

  reference operator()(unsigned int i, unsigned int j) {
    assert(i < nr_points_ && j < d);
    return data_[i * d + j];
  }

  const_reference operator()(unsigned int i, unsigned int j) const {
    assert(i < nr_points_ && j < d);
    return data_[i * d + j];
  }

  iterator Begin() { return data_; }

  iterator End() { return data_ + (nr_points_ * d); }

  const_iterator Begin() const { return data_; }

  const_iterator End() const { return data_ + (nr_points_ * d); }

 protected:
  // continuous 16-byte aligned memory
  T *data_;
  unsigned int nr_points_;
};

template <typename T, unsigned int d>
PtCluster<T, d>::PtCluster() : data_(nullptr), nr_points_(0) {}

template <typename T, unsigned int d>
PtCluster<T, d>::PtCluster(unsigned int n) : data_(nullptr), nr_points_(n) {
  if (n != 0) {
    data_ = IAllocAligned<T>(n * d, 4);
    if (!data_ || !IVerifyAlignment(data_, 4)) {
      nr_points_ = 0;
      IFreeAligned(&data_);
    }
  }
}

template <typename T, unsigned int d>
PtCluster<T, d>::PtCluster(const T *data, unsigned int n)
    : data_(nullptr), nr_points_(n) {
  if (data && n) {
    data_ = IAllocAligned<T>(n * d, 4);
    if (data_ && IVerifyAlignment(data_, 4)) {
      ICopy(data, data_, n * d);
    } else {
      nr_points_ = 0;
      IFreeAligned(&data_);
    }
  }
}

template <typename T, unsigned int d>
void PtCluster<T, d>::CleanUp() {
  IFreeAligned(&data_);
  nr_points_ = 0;
}

template <typename T>
inline int IAssignPointToVoxel(const T *data, T bound_x_min, T bound_x_max,
                               T bound_y_min, T bound_y_max, T bound_z_min,
                               T bound_z_max, T voxel_width_x_rec,
                               T voxel_width_y_rec, int nr_voxel_x,
                               int nr_voxel_y) {
  int i, j, k;
  T x = data[0];
  T y = data[1];
  T z = data[2];

  // points that are outside the defined BBOX are ignored
  if (x < bound_x_min || x > bound_x_max || y < bound_y_min ||
      y > bound_y_max || z < bound_z_min || z > bound_z_max) {
    return (-1);
  }

  // compute the x, y voxel indices
  k = IMin(nr_voxel_x - 1,
           static_cast<int>((x - bound_x_min) * voxel_width_x_rec));
  j = IMin(nr_voxel_y - 1,
           static_cast<int>((y - bound_y_min) * voxel_width_y_rec));
  i = (nr_voxel_x * j) + k;
  return (i);
}

template <typename T>
inline int AssignPointToVoxel(T xx, T yy, T zz, T bound_x_min, T bound_x_max,
                               T bound_y_min, T bound_y_max, T bound_z_min,
                               T bound_z_max, T voxel_width_x_rec,
                               T voxel_width_y_rec, int nr_voxel_x,
                               int nr_voxel_y) {
  int i, j, k;
  T x = xx;
  T y = yy;
  T z = zz;

  // points that are outside the defined BBOX are ignored
  if (x < bound_x_min || x > bound_x_max || y < bound_y_min ||
      y > bound_y_max || z < bound_z_min || z > bound_z_max) {
    return (-1);
  }

  // compute the x, y voxel indices
  k = IMin(nr_voxel_x - 1,
           static_cast<int>((x - bound_x_min) * voxel_width_x_rec));
  j = IMin(nr_voxel_y - 1,
           static_cast<int>((y - bound_y_min) * voxel_width_y_rec));
  i = (nr_voxel_x * j) + k;
  return (i);
}

template <typename T>
class Voxel {
 public:
  Voxel() {}
  ~Voxel() {}
  Voxel(const Voxel<T> &voxel) {
    dim_x_ = voxel.dim_x_;
    dim_y_ = voxel.dim_y_;
    dim_z_ = voxel.dim_z_;
    ix_ = voxel.ix_;
    iy_ = voxel.iy_;
    iz_ = voxel.iz_;
    ICopy3(voxel.v_, v_);
    indices_.assign(voxel.indices_.begin(), voxel.indices_.end());
  }

  Voxel &operator=(const Voxel<T> &voxel) {
    if (this != &voxel) {
      this->dim_x_ = voxel.dim_x_;
      this->dim_y_ = voxel.dim_y_;
      this->dim_z_ = voxel.dim_z_;
      this->ix_ = voxel.ix_;
      this->iy_ = voxel.iy_;
      this->iz_ = voxel.iz_;
      ICopy3(voxel.v_, this->v_);
      this->indices_.assign(voxel.indices_.begin(), voxel.indices_.end());
    }
    return (*this);
  }

  void Init(const T *v, T dim_x, T dim_y, T dim_z, int ix, int iy, int iz) {
    ICopy3(v, v_);
    dim_x_ = dim_x;
    dim_y_ = dim_y;
    dim_z_ = dim_z;
    ix_ = ix;
    iy_ = iy;
    iz_ = iz;
    indices_.clear();
  }

  void Init(T v_x, T v_y, T v_z, T dim_x, T dim_y, T dim_z, int ix, int iy,
            int iz) {
    v_[0] = v_x;
    v_[1] = v_y;
    v_[2] = v_z;
    dim_x_ = dim_x;
    dim_y_ = dim_y;
    dim_z_ = dim_z;
    ix_ = ix;
    iy_ = iy;
    iz_ = iz;
    indices_.clear();
  }

  void Reset() {
    IZero3(v_);
    dim_x_ = dim_y_ = dim_z_ = 0;
    ix_ = iy_ = iz_ = 0;
    indices_.clear();
  }

  void Reserve(unsigned int size) { indices_.reserve(size); }

  void PushBack(int id) { indices_.push_back(id); }

  unsigned int Capacity() const { return (unsigned int)indices_.capacity(); }

  unsigned int NrPoints() const { return (unsigned int)indices_.size(); }

  bool Empty() const { return indices_.empty(); }

  T v_[3], dim_x_, dim_y_, dim_z_;

  // voxel indices in the X, Y, Z dimensions - meaningful for voxel grid
  int ix_, iy_, iz_;

  // point indices
  std::vector<int> indices_;
};

// -----Voxel Grid XY-----
template <typename T>
class VoxelGridXY {
  // assuming at most 320000 points
  static const unsigned int s_nr_max_reserved_points_ = 320000;

 public:
  VoxelGridXY();
  VoxelGridXY(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
              T spatial_bound_x_min, T spatial_bound_x_max,
              T spatial_bound_y_min, T spatial_bound_y_max,
              T spatial_bound_z_min, T spatial_bound_z_max);

  VoxelGridXY &operator=(const VoxelGridXY<T> &vg) {
    if (this != &vg) {
      this->initialized_ = vg.Initialized();
      this->nr_points_ = vg.NrPoints();
      this->nr_point_element_ = vg.NrPointElement();
      this->nr_voxel_x_ = vg.NrVoxelX();
      this->nr_voxel_y_ = vg.NrVoxelY();
      this->nr_voxel_z_ = vg.NrVoxelZ();
      this->data_ = vg.const_data();
      this->semantic_data_ = vg.const_semantic_data();
      vg.GetGridDimension(&this->dim_x_[0], &this->dim_x_[1], &this->dim_y_[0],
                          &this->dim_y_[1], &this->dim_z_[0], &this->dim_z_[1]);
      this->voxels_.resize(vg.NrVoxel());
      for (unsigned int i = 0; i < vg.NrVoxel(); ++i) {
        this->voxels_[i] = vg[i];
      }

      this->AllocAlignedMemory();
    }
    return (*this);
  }

  void CleanUp();

  ~VoxelGridXY() {
    IFreeAligned<float>(&mem_aligned16_f32_);
    IFreeAligned<int>(&mem_aligned16_i32_);
    CleanUp();
  }

  bool Alloc(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
             T spatial_bound_x_min, T spatial_bound_x_max,
             T spatial_bound_y_min, T spatial_bound_y_max,
             T spatial_bound_z_min, T spatial_bound_z_max);

  bool Set(const T *data, const int *semantic_data, unsigned int nr_points,
           unsigned int nr_point_element);

  // sse2 version: only for float type input Data
  bool SetS(const float *data, const int *semantic_data,
      unsigned int nr_points, unsigned int nr_point_element);

  bool Set(const T *data, const int *semantic_data,
      unsigned int nr_points, unsigned int nr_point_element,
           unsigned int nr_voxel_x, unsigned int nr_voxel_y,
           T spatial_bound_x_min, T spatial_bound_x_max, T spatial_bound_y_min,
           T spatial_bound_y_max, T spatial_bound_z_min, T spatial_bound_z_max,
           bool force_bound = true);

  bool Initialized() const { return initialized_; }

  unsigned int NrVoxel() const { return (unsigned int)voxels_.size(); }

  unsigned int NrVoxelX() const { return nr_voxel_x_; }

  unsigned int NrVoxelY() const { return nr_voxel_y_; }

  unsigned int NrVoxelZ() const { return (1); }

  unsigned int NrPoints() const { return nr_points_; }

  unsigned int NrPointElement() const { return nr_point_element_; }

  unsigned int NrIndexedPoints() const;

  std::vector<Voxel<T>> &GetVoxels() { return voxels_; }

  const std::vector<Voxel<T>> &GetConstVoxels() const { return voxels_; }

  void SetVoxels(const std::vector<Voxel<T>> &voxels) {
    voxels_.assign(voxels.begin(), voxels.end());
  }

  bool GetGridDimension(T *dim_min_x, T *dim_max_x, T *dim_min_y, T *dim_max_y,
                        T *dim_min_z, T *dim_max_z) const {
    if (!initialized_) {
      dim_min_x = dim_max_x = dim_min_y = dim_max_y = dim_min_z = dim_max_z =
          static_cast<T>(0);
      return (false);
    }
    *dim_min_x = dim_x_[0];
    *dim_max_x = dim_x_[1];
    *dim_min_y = dim_y_[0];
    *dim_max_y = dim_y_[1];
    *dim_min_z = dim_z_[0];
    *dim_max_z = dim_z_[1];
    return (true);
  }

  void SetGridDimension(T dim_min_x, T dim_max_x, T dim_min_y, T dim_max_y,
                        T dim_min_z, T dim_max_z) {
    dim_x_[0] = dim_min_x;
    dim_x_[1] = dim_max_x;
    dim_y_[0] = dim_min_y;
    dim_y_[1] = dim_max_y;
    dim_z_[0] = dim_min_z;
    dim_z_[1] = dim_max_z;
  }

  bool GetVoxelDimension(T *voxel_width_x, T *voxel_width_y,
                         T *voxel_width_z) const {
    if (!initialized_) {
      *voxel_width_x = *voxel_width_y = *voxel_width_z = static_cast<T>(0);
      return (false);
    }
    *voxel_width_x = voxel_dim_[0];
    *voxel_width_y = voxel_dim_[1];
    *voxel_width_z = voxel_dim_[2];
    return (true);
  }

  void SetVoxelDimension(T voxel_width_x, T voxel_width_y, T voxel_width_z) {
    voxel_dim_[0] = voxel_width_x;
    voxel_dim_[1] = voxel_width_y;
    voxel_dim_[2] = voxel_width_z;
  }

  void SetNrPoints(unsigned int nr_points) { nr_points_ = nr_points; }

  void SetNrPointElement(unsigned int nr_point_element) {
    nr_point_element_ = nr_point_element;
  }

  void SetNrVoxel(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
                  unsigned int nr_voxel_z) {
    nr_voxel_x_ = nr_voxel_x;
    nr_voxel_y_ = nr_voxel_y;
    nr_voxel_z_ = nr_voxel_z;
  }

  void SetPointCloudsData(const T *data) { data_ = data; }

  void SetSematicValueData(const int *data) { semantic_data_ = data; }

  void SetInitialized(bool initialized) { initialized_ = initialized; }

  // int GetAllIndices(std::vector<int> &indices) const;

  int WhichVoxel(T x, T y, T z) const;

  //  add by Fangzhen Li on 03/27/17
  bool GetVoxelCoordinateXY(T x, T y, int *row, int *col) const;

  Voxel<T> &operator[](unsigned int i) {
    assert(i >= 0 && i < voxels_.size());
    return (voxels_[i]);
  }

  const Voxel<T> &operator[](unsigned int i) const {
    assert(i >= 0 && i < voxels_.size());
    return (voxels_[i]);
  }

  Voxel<T> &operator[](int i) {
    assert(i >= 0 && i < static_cast<int>(voxels_.size()));
    return (voxels_[i]);
  }
  const Voxel<T> &operator[](int i) const {
    assert(i >= 0 && i < static_cast<int>(voxels_.size()));
    return (voxels_[i]);
  }

  Voxel<T> &operator()(unsigned int iy, unsigned int ix) {
    assert(iy < nr_voxel_y_ && ix < nr_voxel_x_);
    unsigned int i = nr_voxel_x_ * iy + ix;
    assert(i < voxels_.size());
    return (voxels_[i]);
  }

  const Voxel<T> &operator()(unsigned int iy, unsigned int ix) const {
    assert(iy < nr_voxel_y_ && ix < nr_voxel_x_);
    unsigned int i = nr_voxel_x_ * iy + ix;
    assert(i < voxels_.size());
    return (voxels_[i]);
  }

  T *Data() { return (data_); }

  const T *const_data() const { return (data_); }

  int *SemanticData() { return (semantic_data_); }

  const int *const_semantic_data() const { return (semantic_data_); }

 private:
  void Reserve();
  bool AllocAlignedMemory();

 private:
  unsigned int nr_points_ = 0;
  unsigned int nr_point_element_ = 0;
  unsigned int nr_voxel_x_ = 0;
  unsigned int nr_voxel_y_ = 0;
  unsigned int nr_voxel_z_ = 0;
  const T *data_ = nullptr;  // point clouds memory
  const int* semantic_data_ = nullptr; // semantic value memory
  bool initialized_ = false;
  T dim_x_[2], dim_y_[2], dim_z_[2], voxel_dim_[3];
  float *mem_aligned16_f32_ = nullptr;
  int *mem_aligned16_i32_ = nullptr;
  std::vector<Voxel<T>> voxels_;
};

template <typename T>
VoxelGridXY<T>::VoxelGridXY()
    : nr_points_(0),
      nr_point_element_(0),
      nr_voxel_x_(0),
      nr_voxel_y_(0),
      nr_voxel_z_(1),
      data_(nullptr),
      semantic_data_(nullptr),
      initialized_(false),
      mem_aligned16_f32_(nullptr),
      mem_aligned16_i32_(nullptr) {
  IZero2(dim_x_);
  IZero2(dim_y_);
  IZero2(dim_z_);
  IZero3(voxel_dim_);
  AllocAlignedMemory();
}

template <typename T>
void VoxelGridXY<T>::CleanUp() {
  initialized_ = false;
  data_ = nullptr;
  semantic_data_ = nullptr;
  nr_points_ = nr_point_element_ = nr_voxel_x_ = nr_voxel_y_ = 0;
  nr_voxel_z_ = 1;
  IZero2(dim_x_);
  IZero2(dim_y_);
  IZero2(dim_z_);
  IZero3(voxel_dim_);
  voxels_.clear();
}

template <typename T>
VoxelGridXY<T>::VoxelGridXY(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
                            T spatial_bound_x_min, T spatial_bound_x_max,
                            T spatial_bound_y_min, T spatial_bound_y_max,
                            T spatial_bound_z_min, T spatial_bound_z_max) {
  Alloc(nr_voxel_x, nr_voxel_y, spatial_bound_x_min, spatial_bound_x_max,
        spatial_bound_y_min, spatial_bound_y_max, spatial_bound_z_min,
        spatial_bound_z_max);
  AllocAlignedMemory();
  initialized_ = false;
}

template <typename T>
void VoxelGridXY<T>::Reserve() {
  int r, c, i = 0, m = 0;
  unsigned int n = (nr_voxel_x_ * nr_voxel_y_);

  if (n == 0 || voxels_.size() != n) {
    return;
  }

  double cen_x = (static_cast<double>(nr_voxel_x_ - 1)) * 0.5;
  double cen_y = (static_cast<double>(nr_voxel_y_ - 1)) * 0.5;

  // normalization factor = 1/(2sigma*sigma)
  // hardcode sigma 10.0
  double sigma = static_cast<double>(IAverage(nr_voxel_x_, nr_voxel_y_) / 10.0);
  double nf = IDiv(0.5, (sigma * sigma));
  double dr, drsqr, dc, dcsqr, v, ksum = 0.0;
  std::vector<double> kernel(n, 0.0);

  // pre-compute kernel
  for (r = 0; r < static_cast<int>(nr_voxel_y_); ++r) {
    dr = static_cast<double>(r) - cen_y;
    drsqr = ISqr(dr);
    for (c = 0; c < static_cast<int>(nr_voxel_x_); ++c) {
      dc = static_cast<double>(c) - cen_x;
      dcsqr = ISqr(dc);
      v = IExp(-(drsqr + dcsqr) * nf);
      ksum += v;
      kernel[i++] = v;
    }
  }

  // normalize the kernel, sum to 1.0
  v = IRec(ksum);
  for (i = 0; i < static_cast<int>(n); ++i) {
    kernel[i] *= v;
  }

  // Alloc at least 8 positions
  for (i = 0; i < static_cast<int>(n); ++i) {
    m = IMax(8, static_cast<int>(s_nr_max_reserved_points_ * kernel[i]));
    voxels_[i].Reserve(m);
  }
}

template <typename T>
bool VoxelGridXY<T>::AllocAlignedMemory() {
  if (!mem_aligned16_f32_) {
    mem_aligned16_f32_ = IAllocAligned<float>(4, 4);
  }
  if (!mem_aligned16_i32_) {
    mem_aligned16_i32_ = IAllocAligned<int>(4, 4);
  }
  return (mem_aligned16_f32_ != nullptr && mem_aligned16_i32_ != nullptr &&
          IVerifyAlignment(mem_aligned16_f32_, 4) &&
          IVerifyAlignment(mem_aligned16_i32_, 4));
}

template <typename T>
int VoxelGridXY<T>::WhichVoxel(T x, T y, T z) const {
  int j, k;
  if (!initialized_) {
    return (-1);
  }

  if (x < dim_x_[0] || x > dim_x_[1] || y < dim_y_[0] || y > dim_y_[1] ||
      z < dim_z_[0] || z > dim_z_[1]) {
    return (-1);  // points that are outside the defined BBOX are ignored
  }

  k = static_cast<int>(IMax(
      (unsigned int)0,
      IMin(nr_voxel_x_ - 1, (unsigned int)((x - dim_x_[0]) / voxel_dim_[0]))));
  j = static_cast<int>(IMax(
      (unsigned int)0,
      IMin(nr_voxel_y_ - 1, (unsigned int)((y - dim_y_[0]) / voxel_dim_[1]))));
  return (nr_voxel_x_ * j + k);
}

//  add by Fangzhen Li on 03/27/17
template <typename T>
bool VoxelGridXY<T>::GetVoxelCoordinateXY(T x, T y, int *row, int *col) const {
  if (x < dim_x_[0] || x > dim_x_[1] || y < dim_y_[0] || y > dim_y_[1]) {
    return false;  // points that are outside the defined BBOX are ignored
  }

  *col = static_cast<int>((x - dim_x_[0]) / voxel_dim_[0]);
  *row = static_cast<int>((y - dim_y_[0]) / voxel_dim_[1]);
  return true;
}

template <typename T>
bool VoxelGridXY<T>::Alloc(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
                           T spatial_bound_x_min, T spatial_bound_x_max,
                           T spatial_bound_y_min, T spatial_bound_y_max,
                           T spatial_bound_z_min, T spatial_bound_z_max) {
  if (!nr_voxel_x || !nr_voxel_y) {
    initialized_ = false;
    return initialized_;
  }
  nr_voxel_x_ = nr_voxel_x;
  nr_voxel_y_ = nr_voxel_y;

  unsigned int i, j, k, n = 0;
  unsigned int nr_voxel_xy = (nr_voxel_x_ * nr_voxel_y_);
  unsigned int nr_voxel = nr_voxel_xy;

  T vx, vy, vz, voxel_width_x, voxel_width_y, voxel_width_z;

  voxels_.clear();
  voxels_.resize(nr_voxel);

  // voxel grid dimesion is forced to be the manual input
  dim_x_[0] = spatial_bound_x_min;
  dim_x_[1] = spatial_bound_x_max;

  dim_y_[0] = spatial_bound_y_min;
  dim_y_[1] = spatial_bound_y_max;

  dim_z_[0] = spatial_bound_z_min;
  dim_z_[1] = spatial_bound_z_max;

  T span_x = (dim_x_[1] - dim_x_[0]);
  T span_y = (dim_y_[1] - dim_y_[0]);
  T span_z = (dim_z_[1] - dim_z_[0]);

  assert(span_x > 0 && span_y > 0 && span_z > 0);

  voxel_dim_[0] = voxel_width_x = IDiv(span_x, static_cast<int>(nr_voxel_x_));
  voxel_dim_[1] = voxel_width_y = IDiv(span_y, static_cast<int>(nr_voxel_y_));
  voxel_dim_[2] = voxel_width_z = span_z;

  std::vector<T> vxs(nr_voxel_x_);

  vxs[0] = dim_x_[0];
  for (i = 1; i < nr_voxel_x_; i++) {
    vxs[i] = vxs[i - 1] + voxel_width_x;
  }

  vy = dim_y_[0];
  vz = dim_z_[0];

  for (j = 0; j < nr_voxel_y_; j++) {
    for (k = 0; k < nr_voxel_x_; k++) {
      vx = vxs[k];
      voxels_[n++].Init(vx, vy, vz, voxel_width_x, voxel_width_y, voxel_width_z,
                        static_cast<int>(k), static_cast<int>(j), 0);
    }
    vy += voxel_width_y;
  }

  // pre-Alloc capacaity for indices_ vector - speed-up the code slightly
  Reserve();
  return (true);
}

template <typename T>
bool VoxelGridXY<T>::Set(const T *data, const int *semantic_data,
  unsigned int nr_points, unsigned int nr_point_element) {
  if (!data || !nr_points || !nr_point_element || !nr_voxel_x_ ||
      !semantic_data || !nr_voxel_y_ || nr_voxel_z_ != 1) {
    initialized_ = false;
    return initialized_;
  }

  data_ = data;
  semantic_data_ = semantic_data;
  nr_points_ = nr_points;
  nr_point_element_ = nr_point_element;

  unsigned int nr_voxel = (nr_voxel_x_ * nr_voxel_y_);
  unsigned int i, nd = 0;
  int id, n = 0;

  T voxel_width_x_rec = IRec(voxel_dim_[0]);
  T voxel_width_y_rec = IRec(voxel_dim_[1]);

  T bound_x_min = dim_x_[0];
  T bound_x_max = dim_x_[1];
  T bound_y_min = dim_y_[0];
  T bound_y_max = dim_y_[1];
  T bound_z_min = dim_z_[0];
  T bound_z_max = dim_z_[1];

  // clear the old index buffer
  for (i = 0; i < nr_voxel; ++i) {
    voxels_[i].indices_.clear();
  }

  // Assign point indices
  for (; n < static_cast<int>(nr_points_); n++, nd += nr_point_element_) {
    id = IAssignPointToVoxel(
        data_ + nd, bound_x_min, bound_x_max, bound_y_min, bound_y_max,
        bound_z_min, bound_z_max, voxel_width_x_rec, voxel_width_y_rec,
        static_cast<int>(nr_voxel_x_), static_cast<int>(nr_voxel_y_));

    if (id >= 0) {
      voxels_[id].indices_.push_back(n);
    }
  }

  initialized_ = true;
  return (initialized_);
}

template <typename T>
bool VoxelGridXY<T>::SetS(const float *data, const int *semantic_data,
  unsigned int nr_points, unsigned int nr_point_element) {
  if (!data || !nr_points || !nr_point_element || !nr_voxel_x_ ||
      !semantic_data || !nr_voxel_y_ || nr_voxel_z_ != 1) {
    initialized_ = false;
    return initialized_;
  }

  data_ = data;
  semantic_data_ = semantic_data;
  nr_points_ = nr_points;
  nr_point_element_ = nr_point_element;

  unsigned int nr_voxel = (nr_voxel_x_ * nr_voxel_y_);
  unsigned int i, n = 0;
  int id;

  float voxel_width_x_rec = IRec(static_cast<float>(voxel_dim_[0]));
  float voxel_width_y_rec = IRec(static_cast<float>(voxel_dim_[1]));

  __m128i iv_nr_voxel_x = _mm_setr_epi32(static_cast<int>(nr_voxel_x_), 0,
                                         static_cast<int>(nr_voxel_x_), 0);
  __m128i iv_nr_voxel_x_m1 = _mm_set1_epi32(static_cast<int>(nr_voxel_x_ - 1));
  __m128i iv_nr_voxel_y_m1 = _mm_set1_epi32(static_cast<int>(nr_voxel_y_ - 1));

  __m128 v_width_x_rec = _mm_set_ps1(voxel_width_x_rec);
  __m128 v_width_y_rec = _mm_set_ps1(voxel_width_y_rec);

  __m128 v_x_min = _mm_set_ps1(dim_x_[0]);
  __m128 v_x_max = _mm_set_ps1(dim_x_[1]);
  __m128 v_y_min = _mm_set_ps1(dim_y_[0]);
  __m128 v_y_max = _mm_set_ps1(dim_y_[1]);
  __m128 v_z_min = _mm_set_ps1(dim_z_[0]);
  __m128 v_z_max = _mm_set_ps1(dim_z_[1]);

  __m128 v_cmp_x, v_cmp_y, v_cmp_z, v_in_roi;
  v_in_roi = _mm_setr_ps(1.0, 1.0, 1.0, 1.0);

  __m128 v_xs, v_ys, v_zs;
  __m128i iv_indices, iv_x_indices, iv_y_indices, iv_v_indices_02,
      iv_v_indices_13;

  for (i = 0; i < nr_voxel; ++i) {
    voxels_[i].indices_.clear();
  }

  unsigned int nr_loops = (nr_points >> 2);
  unsigned int nr_fast_processed = (nr_loops << 2);
  unsigned int remainder = nr_points - nr_fast_processed;

  unsigned int d1 = nr_point_element;
  unsigned int d2 = (nr_point_element << 1);
  unsigned int d3 = d1 + d2;
  unsigned int d4 = (nr_point_element << 2);

  // xyz are required to be stored in continuous memory
  const float *cptr_x = data;
  const float *cptr_y = data + 1;
  const float *cptr_z = data + 2;
  const float *cptr_remainder = data + (nr_fast_processed * nr_point_element);

  for (i = 0; i < nr_loops; ++i, n += 4) {
    // Set memory
    v_xs = _mm_setr_ps(cptr_x[0], cptr_x[d1], cptr_x[d2], cptr_x[d3]);
    v_ys = _mm_setr_ps(cptr_y[0], cptr_y[d1], cptr_y[d2], cptr_y[d3]);
    v_zs = _mm_setr_ps(cptr_z[0], cptr_z[d1], cptr_z[d2], cptr_z[d3]);

    // compare range:
    v_cmp_x =
        _mm_and_ps(_mm_cmpge_ps(v_xs, v_x_min), _mm_cmple_ps(v_xs, v_x_max));
    v_cmp_y =
        _mm_and_ps(_mm_cmpge_ps(v_ys, v_y_min), _mm_cmple_ps(v_ys, v_y_max));
    v_cmp_z =
        _mm_and_ps(_mm_cmpge_ps(v_zs, v_z_min), _mm_cmple_ps(v_zs, v_z_max));
    v_in_roi = _mm_and_ps(_mm_and_ps(v_cmp_x, v_cmp_y), v_cmp_z);

    // vector operations, cast into signed integers
    v_xs = _mm_sub_ps(v_xs, v_x_min);
    v_xs = _mm_mul_ps(v_xs, v_width_x_rec);
    iv_x_indices = _mm_cvttps_epi32(v_xs);  // truncate towards zero
    iv_x_indices = _mm_min_epi32(iv_nr_voxel_x_m1, iv_x_indices);

    // vector operations, cast into signed integers
    v_ys = _mm_sub_ps(v_ys, v_y_min);
    v_ys = _mm_mul_ps(v_ys, v_width_y_rec);
    iv_y_indices = _mm_cvttps_epi32(v_ys);  // truncate towards zero
    iv_y_indices = _mm_min_epi32(iv_nr_voxel_y_m1, iv_y_indices);

    iv_v_indices_02 = _mm_mullo_epi32(iv_y_indices, iv_nr_voxel_x);
    iv_y_indices = _mm_shuffle_epi32(iv_y_indices, _MM_SHUFFLE(2, 3, 0, 1));

    iv_v_indices_13 = _mm_mullo_epi32(iv_y_indices, iv_nr_voxel_x);
    iv_v_indices_13 =
        _mm_shuffle_epi32(iv_v_indices_13, _MM_SHUFFLE(2, 3, 0, 1));

    iv_indices = _mm_add_epi32(iv_v_indices_02, iv_v_indices_13);
    iv_indices = _mm_add_epi32(iv_indices, iv_x_indices);

    // store values from registers to memory
    // address 16byte-aligned
    _mm_store_ps(mem_aligned16_f32_, v_in_roi);
    // address 16byte-aligned
    _mm_store_si128(reinterpret_cast<__m128i *>(mem_aligned16_i32_),
                    iv_indices);

    if (mem_aligned16_f32_[0] != 0) {
      voxels_[mem_aligned16_i32_[0]].indices_.push_back(n);
    }
    if (mem_aligned16_f32_[1] != 0) {
      voxels_[mem_aligned16_i32_[1]].indices_.push_back(n + 1);
    }
    if (mem_aligned16_f32_[2] != 0) {
      voxels_[mem_aligned16_i32_[2]].indices_.push_back(n + 2);
    }
    if (mem_aligned16_f32_[3] != 0) {
      voxels_[mem_aligned16_i32_[3]].indices_.push_back(n + 3);
    }
    cptr_x += d4;
    cptr_y += d4;
    cptr_z += d4;
  }

  // handling remaining points
  for (i = 0; i < remainder; i++, n++) {
    id = IAssignPointToVoxel(cptr_remainder, dim_x_[0], dim_x_[1], dim_y_[0],
                             dim_y_[1], dim_z_[0], dim_z_[1], voxel_width_x_rec,
                             voxel_width_y_rec, static_cast<int>(nr_voxel_x_),
                             static_cast<int>(nr_voxel_y_));

    if (id >= 0) {
      voxels_[id].indices_.push_back(n);
    }

    cptr_remainder += nr_point_element;
  }

  // for (i = 0; i < nr_voxel; ++i) {
  //     voxels_[i].indices_.clear();
  // }
  // for (i = 0; i < nr_points; i++, n++) {
  //     float x = data_[nr_point_element_ * i];
  //     float y = data_[nr_point_element_ * i + 1];
  //     float z = data_[nr_point_element_ * i + 2];
  //     id = AssignPointToVoxel(x, y, z, dim_x_[0], dim_x_[1], dim_y_[0], dim_y_[1], dim_z_[0], dim_z_[1], 
  //         voxel_width_x_rec, voxel_width_y_rec, static_cast<int>(nr_voxel_x_), static_cast<int>(nr_voxel_y_));
      
  //     if (id >= 0) {
  //         voxels_[id].indices_.push_back(i);
  //     }
  // }

  initialized_ = true;
  return (initialized_);
}

template <typename T>
bool VoxelGridXY<T>::Set(const T *data, const int *semantic_data,
  unsigned int nr_points, unsigned int nr_point_element,
  unsigned int nr_voxel_x, unsigned int nr_voxel_y, T spatial_bound_x_min,
  T spatial_bound_x_max, T spatial_bound_y_min, T spatial_bound_y_max,
  T spatial_bound_z_min, T spatial_bound_z_max, bool force_bound) {
  if (!data || !nr_points || !nr_point_element || !nr_voxel_x || !nr_voxel_y) {
    initialized_ = false;
    return initialized_;
  }

  data_ = data;
  semantic_data_ = semantic_data;
  nr_points_ = nr_points;
  nr_point_element_ = nr_point_element;
  nr_voxel_x_ = nr_voxel_x;
  nr_voxel_y_ = nr_voxel_y;
  nr_voxel_z_ = 1;

  unsigned int i, j, k, n = 0, nd = 0;
  unsigned int nr_voxel_xy = (nr_voxel_x_ * nr_voxel_y_);
  unsigned int nr_voxel = nr_voxel_xy;
  unsigned int nr_voxel_xm1 = nr_voxel_x - 1;
  unsigned int nr_voxel_ym1 = nr_voxel_y - 1;
  T vx, vy, vz, x, y, z;
  T voxel_width_x, voxel_width_y, voxel_width_z;

  voxels_.clear();
  voxels_.resize(nr_voxel);

  // if force_bound then the voxel grid dimesion is forced to be the manual
  //  * input
  if (force_bound) {
    dim_x_[0] = spatial_bound_x_min;
    dim_x_[1] = spatial_bound_x_max;

    dim_y_[0] = spatial_bound_y_min;
    dim_y_[1] = spatial_bound_y_max;

    dim_z_[0] = spatial_bound_z_min;
    dim_z_[1] = spatial_bound_z_max;
  } else {
    IGetPointcloudsDimWBound(
        data_, nr_points_, 0, nr_point_element_, &dim_x_[0], &dim_x_[1],
        &dim_y_[0], &dim_y_[1], &dim_z_[0], &dim_z_[1], spatial_bound_x_min,
        spatial_bound_x_max, spatial_bound_y_min, spatial_bound_y_max,
        spatial_bound_z_min, spatial_bound_z_max);
  }

  T dim_x_min = dim_x_[0];
  T dim_x_max = dim_x_[1];

  T dim_y_min = dim_y_[0];
  T dim_y_max = dim_y_[1];

  T dim_z_min = dim_z_[0];
  T dim_z_max = dim_z_[1];

  T span_x = (dim_x_max - dim_x_min);
  T span_y = (dim_y_max - dim_y_min);
  T span_z = (dim_z_max - dim_z_min);

  assert(span_x > 0 && span_y > 0 && span_z > 0);

  voxel_dim_[0] = voxel_width_x = IDiv(span_x, static_cast<int>(nr_voxel_x_));
  voxel_dim_[1] = voxel_width_y = IDiv(span_y, static_cast<int>(nr_voxel_y_));
  voxel_dim_[2] = voxel_width_z = span_z;

  T voxel_width_x_rec = IRec(voxel_dim_[0]);
  T voxel_width_y_rec = IRec(voxel_dim_[1]);

  std::vector<T> vxs(nr_voxel_x_);
  vxs[0] = dim_x_[0];

  for (i = 1; i < nr_voxel_x_; i++) {
    vxs[i] = vxs[i - 1] + voxel_width_x;
  }

  vy = dim_y_[0];
  vz = dim_z_[0];

  for (j = 0; j < nr_voxel_y_; j++) {
    for (k = 0; k < nr_voxel_x_; k++) {
      vx = vxs[k];
      voxels_[n++].init(vx, vy, vz, voxel_width_x, voxel_width_y, voxel_width_z,
                        static_cast<int>(k), static_cast<int>(j), 0);
    }
    vy += voxel_width_y;
  }

  // Assign point indices
  for (; n < nr_points_; n++, nd += nr_point_element_) {
    x = data_[nd];
    y = data_[nd + 1];
    z = data_[nd + 2];

    if (x < dim_x_min || x > dim_x_max || y < dim_y_min || y > dim_y_max ||
        z < dim_z_min || z > dim_z_max) {
      // points that are outside the defined BBOX are ignored
      continue;
    }

    k = IMax((unsigned int)0,
             IMin(nr_voxel_xm1,
                  (unsigned int)((x - dim_x_min) * voxel_width_x_rec)));
    j = IMax((unsigned int)0,
             IMin(nr_voxel_ym1,
                  (unsigned int)((y - dim_y_min) * voxel_width_y_rec)));
    i = nr_voxel_x * j + k;

    assert(i < nr_voxel);
    voxels_[i].indices_.push_back(n);
  }
  initialized_ = true;
  return initialized_;
}

// template <typename T>
// int VoxelGridXY<T>::GetAllIndices(std::vector<int> &indices) const {
//   unsigned int i, j;
//   indices.clear();

//   if (!initialized_) {
//     return -1;
//   }
//   for (i = 0; i < voxels_.size(); ++i) {
//     for (j = 0; j < voxels_[i].indices_.size(); ++j) {
//       indices.push_back(voxels_[i].indices_.at(j));
//     }
//   }

//   return static_cast<int> (indices.size());
// }

template <typename T>
unsigned int VoxelGridXY<T>::NrIndexedPoints() const {
  if (!initialized_) {
    return 0;
  }
  unsigned int i, n = 0;
  for (i = 0; i < voxels_.size(); ++i) {
    n += voxels_[i].indices_.size();
  }
  return n;
}

template <typename T>
static void IPushBackVectors(const std::vector<T> &src, std::vector<T> *dst) {
  unsigned int i = src.size();
  unsigned int j = dst->size();

  if (i == 0) {
    return;
  } else if (j == 0) {
    dst->assign(src.begin(), src.end());
  } else {
    dst->resize(i + j);
    ICopy(src.data(), dst->data() + j, static_cast<int>(i));
  }
}

// VoxelgridXY downsample functions
template <typename T>
bool IDownsampleVoxelGridXY(const VoxelGridXY<T> &src, VoxelGridXY<T> *dst,
                            unsigned int dsf_dim_x, unsigned int dsf_dim_y) {
  if (!src.Initialized()) {
    return (false);
  } else if (dsf_dim_x == 0 && dsf_dim_y == 0) {
    *dst = src;
    return (true);
  } else {
    if (&src == dst) {
      return (false);  // not allowed to downsample a voxel grid from itself
    }
  }

  dst->CleanUp();  // perform CleanUp first

  // # of voxels of the src voxel grid
  unsigned int nr_voxel_x_src = src.NrVoxelX();
  unsigned int nr_voxel_y_src = src.NrVoxelY();
  unsigned int nr_voxel_z_src = src.NrVoxelZ();

  assert(nr_voxel_z_src == 1);

  // scale factors
  unsigned int sf_x = (unsigned int)IPow((unsigned int)2, dsf_dim_x);
  unsigned int sf_y = (unsigned int)IPow((unsigned int)2, dsf_dim_y);

  // compute the # of voxels for the new scale
  unsigned int nr_voxel_x_dst = nr_voxel_x_src / sf_x;
  unsigned int nr_voxel_y_dst = nr_voxel_y_src / sf_y;
  unsigned int nr_voxel_z_dst = nr_voxel_z_src;

  if (!nr_voxel_x_dst || !nr_voxel_y_dst || !nr_voxel_z_dst) {
    return (false);  // do not continue
  }

  unsigned int nr_res_voxel_x_dst = (nr_voxel_x_src % sf_x);
  unsigned int nr_res_voxel_y_dst = (nr_voxel_y_src % sf_y);

  T voxel_width_x_src = 0;
  T voxel_width_y_src = 0;
  T voxel_width_z_src = 0;
  T dim_min_x_src = 0;
  T dim_max_x_src = 0;
  T dim_min_y_src = 0;
  T dim_max_y_src = 0;
  T dim_min_z_src = 0;
  T dim_max_z_src = 0;

  // the dimension of the voxels in the src voxel grid
  src.GetVoxelDimension(&voxel_width_x_src, &voxel_width_y_src,
                        &voxel_width_z_src);

  src.GetGridDimension(&dim_min_x_src, &dim_max_x_src, &dim_min_y_src,
                       &dim_max_y_src, &dim_min_z_src, &dim_max_z_src);

  // new voxel dimensions after downsampling the 3D grid
  T voxel_width_x_dst = (sf_x * voxel_width_x_src);
  T voxel_width_y_dst = (sf_y * voxel_width_y_src);
  T voxel_width_z_dst = voxel_width_z_src;

  // new grid dimensions after downsampling the 3D grid
  T dim_x_dst[2], dim_y_dst[2], dim_z_dst[2];

  dim_x_dst[0] = dim_min_x_src;
  dim_y_dst[0] = dim_min_y_src;
  dim_z_dst[0] = dim_min_z_src;

  dim_x_dst[1] = (0 == nr_res_voxel_x_dst)
                     ? dim_max_x_src
                     : (voxel_width_x_dst * nr_voxel_x_dst + dim_x_dst[0]);
  dim_y_dst[1] = (0 == nr_res_voxel_y_dst)
                     ? dim_max_y_src
                     : (voxel_width_y_dst * nr_voxel_y_dst + dim_y_dst[0]);
  dim_z_dst[1] = dim_max_z_src;

  // Set dst
  dst->SetNrPoints(src.NrPoints());
  dst->SetNrPointElement(src.NrPointElement());
  dst->SetVoxelDimension(voxel_width_x_dst, voxel_width_y_dst,
                         voxel_width_z_dst);
  dst->SetGridDimension(dim_x_dst[0], dim_x_dst[1], dim_y_dst[0], dim_y_dst[1],
                        dim_z_dst[0], dim_z_dst[1]);
  dst->SetNrVoxel(nr_voxel_x_dst, nr_voxel_y_dst, nr_voxel_z_dst);
  dst->SetPointCloudsData(src.const_data());

  unsigned int i, j, r, c, rs, cs, rspi, n = 0;

  T vy, vz = dim_z_dst[0];
  std::vector<T> vxs(nr_voxel_x_dst);

  vxs[0] = dim_x_dst[0];

  for (i = 1; i < nr_voxel_x_dst; i++) {
    vxs[i] = vxs[i - 1] + voxel_width_x_dst;
  }

  std::vector<Voxel<T>> voxels(nr_voxel_y_dst * nr_voxel_x_dst);

  for (r = 0; r < nr_voxel_y_dst; r++) {
    rs = (r * sf_y);
    vy = dim_y_dst[0] + (r * voxel_width_y_dst);

    for (c = 0; c < nr_voxel_x_dst; c++) {
      cs = (c * sf_x);
      voxels[n].init(vxs[c], vy, vz, voxel_width_x_dst, voxel_width_y_dst,
                     voxel_width_z_dst, static_cast<int>(c),
                     static_cast<int>(r), static_cast<int>(0));
      // collect samples from its lower scale:
      for (i = 0; i < sf_y; ++i) {
        rspi = rs + i;
        for (j = 0; j < sf_x; ++j) {
          IPushBackVectors(src(rspi, cs + j).indices_, &voxels[n].indices_);
        }
      }
      // increase voxel index by 1
      n++;
    }
  }
  dst->SetVoxels(voxels);
  dst->SetInitialized(true);
  return (true);
}

// -----Multiscale Voxel Grid Pyramid-----
template <typename DATA_TYPE>
class VoxelGridXYPyramid {
 public:
  VoxelGridXYPyramid();
  VoxelGridXYPyramid(unsigned int nr_scale, unsigned int nr_voxel_x_base,
                     unsigned int nr_voxel_y_base, unsigned int dsf_x,
                     unsigned int dsf_y, DATA_TYPE spatial_bound_x_min,
                     DATA_TYPE spatial_bound_x_max,
                     DATA_TYPE spatial_bound_y_min,
                     DATA_TYPE spatial_bound_y_max,
                     DATA_TYPE spatial_bound_z_min,
                     DATA_TYPE spatial_bound_z_max) {
    Alloc(nr_scale, nr_voxel_x_base, nr_voxel_y_base, dsf_x, dsf_y,
          spatial_bound_x_min, spatial_bound_x_max, spatial_bound_y_min,
          spatial_bound_y_max, spatial_bound_z_min, spatial_bound_z_max);
  }

  void CleanUp();

  ~VoxelGridXYPyramid() { CleanUp(); }

  bool Alloc(unsigned int nr_scale, unsigned int nr_voxel_x_base,
             unsigned int nr_voxel_y_base, unsigned int dsf_x,
             unsigned int dsf_y, DATA_TYPE spatial_bound_x_min,
             DATA_TYPE spatial_bound_x_max, DATA_TYPE spatial_bound_y_min,
             DATA_TYPE spatial_bound_y_max, DATA_TYPE spatial_bound_z_min,
             DATA_TYPE spatial_bound_z_max);

  // non-sse2 version
  bool Set(const DATA_TYPE *pc, unsigned int nr_points,
           unsigned int nr_point_element);

  // the sse2 version - speed up the voxel grid construction, for float type
  // * points only
  bool SetS(const float *pc, unsigned int nr_points,
            unsigned int nr_point_element);

  bool Set(const DATA_TYPE *pc, unsigned int nr_scale, unsigned int nr_points,
           unsigned int nr_point_element, unsigned int nr_voxel_x_base,
           unsigned int nr_voxel_y_base, unsigned int dsf_x, unsigned int dsf_y,
           DATA_TYPE spatial_bound_x_min, DATA_TYPE spatial_bound_x_max,
           DATA_TYPE spatial_bound_y_min, DATA_TYPE spatial_bound_y_max,
           DATA_TYPE spatial_bound_z_min, DATA_TYPE spatial_bound_z_max);

  unsigned int NrScale() const { return (unsigned int)vgrids_.size(); }

  unsigned int NrVoxel(unsigned int i = 0) const {
    return (i < vgrids_.size()) ? vgrids_[i].nr_voxel() : 0;
  }

  unsigned int NrVoxelX(unsigned int i = 0) const {
    return (i < vgrids_.size()) ? vgrids_[i].nr_voxel_x() : 0;
  }

  unsigned int NrVoxelY(unsigned int i = 0) const {
    return (i < vgrids_.size()) ? vgrids_[i].nr_voxel_y() : 0;
  }

  unsigned int NrVoxelZ(unsigned int i = 0) const {
    return (i < vgrids_.size()) ? vgrids_[i].nr_voxel_z() : 0;
  }

  unsigned int NrPoints(unsigned int scale) const {
    return (scale < vgrids_.size()) ? vgrids_[scale].nr_points() : 0;
  }

  unsigned int NrPoints() const { return nr_points_; }

  unsigned int NrPointElement() const { return nr_point_element_; }

  bool Initialized() const;

  unsigned int GetDsfX() const { return dsf_x_; }
  unsigned int GetDsfY() const { return dsf_y_; }
  unsigned int GetDsfZ() const { return (0); }

  VoxelGridXY<DATA_TYPE> &operator[](unsigned int i) {
    assert(i >= 0 && i < vgrids_.size());
    return vgrids_[i];
  }

  const VoxelGridXY<DATA_TYPE> &operator[](unsigned int i) const {
    assert(i >= 0 && i < vgrids_.size());
    return vgrids_[i];
  }

  VoxelGridXY<DATA_TYPE> &operator[](int i) {
    assert(i >= 0 && i < static_cast<int>(vgrids_.size()));
    return vgrids_[i];
  }

  const VoxelGridXY<DATA_TYPE> &operator[](int i) const {
    assert(i >= 0 && i < static_cast<int>(vgrids_.size()));
    return vgrids_[i];
  }

  const DATA_TYPE *ConstData() const { return pc_; }

  DATA_TYPE *Data() { return pc_; }

 private:
  unsigned int nr_points_, nr_point_element_;
  unsigned int dsf_x_, dsf_y_, dsf_z_;
  const DATA_TYPE *pc_;
  std::vector<VoxelGridXY<DATA_TYPE>> vgrids_;
};

template <typename DATA_TYPE>
VoxelGridXYPyramid<DATA_TYPE>::VoxelGridXYPyramid()
    : pc_(nullptr),
      nr_points_(0),
      nr_point_element_(0),
      dsf_x_(0),
      dsf_y_(0),
      dsf_z_(0) {}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::Alloc(
    unsigned int nr_scale, unsigned int nr_voxel_x_base,
    unsigned int nr_voxel_y_base, unsigned int dsf_x, unsigned int dsf_y,
    DATA_TYPE spatial_bound_x_min, DATA_TYPE spatial_bound_x_max,
    DATA_TYPE spatial_bound_y_min, DATA_TYPE spatial_bound_y_max,
    DATA_TYPE spatial_bound_z_min, DATA_TYPE spatial_bound_z_max) {
  if (!nr_scale || !nr_voxel_x_base || !nr_voxel_y_base) {
    return false;
  }

  unsigned int scale;
  unsigned int nr_voxel_x = nr_voxel_x_base;
  unsigned int nr_voxel_y = nr_voxel_y_base;
  unsigned int sf_x = (unsigned int)IPow((unsigned int)2, dsf_x);
  unsigned int sf_y = (unsigned int)IPow((unsigned int)2, dsf_y);

  dsf_x_ = dsf_x;
  dsf_y_ = dsf_y;
  dsf_z_ = 0;

  vgrids_.clear();
  vgrids_.resize(nr_scale);

  for (scale = 0; scale < nr_scale; ++scale) {
    if (!vgrids_[scale].alloc(nr_voxel_x, nr_voxel_y, spatial_bound_x_min,
                              spatial_bound_x_max, spatial_bound_y_min,
                              spatial_bound_y_max, spatial_bound_z_min,
                              spatial_bound_z_max)) {
      break;
    }
    nr_voxel_x /= sf_x;
    nr_voxel_y /= sf_y;
  }

  if (scale == 0) {
    vgrids_.clear();
    return false;
  } else {
    if (scale != nr_scale) {
      vgrids_.resize(scale);
    }
  }

  return (true);
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::Set(const DATA_TYPE *pc,
                                        unsigned int nr_points,
                                        unsigned int nr_point_element) {
  if (!pc || !nr_points || !nr_point_element || vgrids_.empty()) {
    return false;
  }

  unsigned int scale, nr_scale = (unsigned int)vgrids_.size();

  pc_ = pc;
  nr_points_ = nr_points;
  nr_point_element_ = nr_point_element;

  for (scale = 0; scale < nr_scale; ++scale) {
    if (!vgrids_[scale].set(pc, nr_points, nr_point_element)) {
      break;
    }
  }

  // break in the middle - last (NrScale - s) grids are invalid
  if (scale < nr_scale) {
    vgrids_.resize(scale);
  }

  return (vgrids_.size() == nr_scale);
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::SetS(const float *pc,
                                         unsigned int nr_points,
                                         unsigned int nr_point_element) {
  if (!pc || !nr_points || !nr_point_element || vgrids_.empty()) {
    return false;
  }

  unsigned int scale, nr_scale = (unsigned int)vgrids_.size();

  pc_ = pc;
  nr_points_ = nr_points;
  nr_point_element_ = nr_point_element;

  for (scale = 0; scale < nr_scale; ++scale) {
    if (!vgrids_[scale].set_s(pc, nr_points, nr_point_element)) {
      break;
    }
  }

  // break in the middle - last (NrScale - s) grids are invalid
  if (scale < nr_scale) {
    vgrids_.resize(scale);
  }

  return (vgrids_.size() == nr_scale);
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::Set(
    const DATA_TYPE *pc, unsigned int nr_scale, unsigned int nr_points,
    unsigned int nr_point_element, unsigned int nr_voxel_x_base,
    unsigned int nr_voxel_y_base, unsigned int dsf_x, unsigned int dsf_y,
    DATA_TYPE spatial_bound_x_min, DATA_TYPE spatial_bound_x_max,
    DATA_TYPE spatial_bound_y_min, DATA_TYPE spatial_bound_y_max,
    DATA_TYPE spatial_bound_z_min, DATA_TYPE spatial_bound_z_max) {
  if (!pc || !nr_scale || !nr_points || !nr_point_element || !nr_voxel_x_base ||
      !nr_voxel_y_base) {
    return false;
  }

  unsigned int i, s;
  pc_ = pc;
  nr_points_ = nr_points;
  nr_point_element_ = nr_point_element;

  dsf_x_ = dsf_x;
  dsf_y_ = dsf_y;
  dsf_z_ = 0;

  vgrids_.clear();
  vgrids_.resize(nr_scale);

  // Set the base level pyramid
  if (!vgrids_[0].set(pc_, nr_points, nr_point_element, nr_voxel_x_base,
                      nr_voxel_y_base, spatial_bound_x_min, spatial_bound_x_max,
                      spatial_bound_y_min, spatial_bound_y_max,
                      spatial_bound_z_min, spatial_bound_z_max)) {
    vgrids_.clear();
    return false;
  }

  for (s = 1; s < nr_scale; ++s) {
    if (!IDownsampleVoxelGridXY(vgrids_[s - 1], &vgrids_[s], dsf_x_, dsf_y_)) {
      break;
    }
  }

  if (s < nr_scale) {
    for (i = 0; i < (nr_scale - s); ++i) {
      vgrids_.pop_back();
    }
  }

  return (vgrids_.size() == nr_scale);  // check if all scales are valid
}

template <typename DATA_TYPE>
void VoxelGridXYPyramid<DATA_TYPE>::CleanUp() {
  pc_ = nullptr;
  vgrids_.clear();
  nr_points_ = nr_point_element_ = dsf_x_ = dsf_y_ = dsf_z_ = 0;
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::Initialized() const {
  unsigned int i;
  if (vgrids_.empty()) {
    return false;
  }
  for (i = 0; i < vgrids_.size(); ++i) {
    if (!vgrids_[i].initialized()) {
      break;
    }
  }
  return (i == vgrids_.size());
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
