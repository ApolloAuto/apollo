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
#ifndef I_LIB_PC_I_STRUCT_S_H
#define I_LIB_PC_I_STRUCT_S_H

#include <smmintrin.h>
#include <cassert>
#include <vector>
#include "../core/i_alloc.h"
#include "../core/i_blas.h"
#include "i_util.h"

namespace idl {
template <typename T, unsigned int d>
class PtCluster {
 public:
  typedef T* iterator;
  typedef const T* const_iterator;
  typedef T& reference;
  typedef const T& const_reference;

  static unsigned int nr_point_element() { return (d); }

  static unsigned int pointsize_in_byte() { return (d * sizeof(T)); }

  unsigned int clustersize_in_byte() const {
    return (d * sizeof(T) * _nr_points);
  }

  unsigned int nr_points() const { return (_nr_points); }

  bool initialized() const { return (_nr_points > 0); }

  const T* const_data() const { return (_data); }

  T* data() { return (_data); }

  void cleanup();

  PtCluster();
  explicit PtCluster(unsigned int n);
  PtCluster(const T* data, unsigned int n);
  PtCluster(const PtCluster<T, d>& c) : _data(NULL), _nr_points(0) {
    if (c.nr_points()) {
      _nr_points = c.nr_points();
      _data = i_alloc_aligned<T>(_nr_points * d, 4);
      assert(_data != NULL && i_verify_alignment(_data, 4));
      i_copy(c.const_data(), _data, _nr_points * d);
    }
  }

  PtCluster& operator=(const PtCluster<T, d>& c) {
    if (this != &c) {
      if (this->_nr_points != c.nr_points()) {
        i_free_aligned(_data);
        _data = i_alloc_aligned<T>(this->_nr_points * d, 4);
        assert(_data != NULL && i_verify_alignment(_data, 4));
        this->_nr_points = c.nr_points();
      }
      i_copy(c.const_data(), _data, this->_nr_points * d);
    }
    return (*this);
  }

  virtual ~PtCluster() { cleanup(); }

  T* operator[](unsigned int i) {
    assert(i < _nr_points);
    assert(_data != NULL);
    return (_data + i * d);
  }

  const T* operator[](unsigned int i) const {
    assert(i < _nr_points);
    assert(_data != NULL);
    return (_data + i * d);
  }

  reference operator()(unsigned int i, unsigned int j) {
    assert(i < _nr_points && j < d);
    return _data[i * d + j];
  }

  const_reference operator()(unsigned int i, unsigned int j) const {
    assert(i < _nr_points && j < d);
    return _data[i * d + j];
  }

  iterator begin() { return _data; }

  iterator end() { return _data + (_nr_points * d); }

  const_iterator begin() const { return _data; }

  const_iterator end() const { return data + (_nr_points * d); }

 protected:
  /*continuous 16-byte aligned memory*/
  T* _data;
  unsigned int _nr_points;
};

template <typename T, unsigned int d>
PtCluster<T, d>::PtCluster() : _data(NULL), _nr_points(0) {}

template <typename T, unsigned int d>
PtCluster<T, d>::PtCluster(unsigned int n) : _data(NULL), _nr_points(n) {
  if (n != 0) {
    _data = i_alloc_aligned<T>(n * d, 4);
    if (!_data || !i_verify_alignment(_data, 4)) {
      _nr_points = 0;
      i_free_aligned(_data);
    }
  }
}

template <typename T, unsigned int d>
PtCluster<T, d>::PtCluster(const T* data, unsigned int n)
    : _data(NULL), _nr_points(n) {
  if (data && n) {
    _data = i_alloc_aligned<T>(n * d, 4);
    if (_data && i_verify_alignment(_data, 4)) {
      i_copy(data, _data, n * d);
    } else {
      _nr_points = 0;
      i_free_aligned(_data);
    }
  }
}

template <typename T, unsigned int d>
void PtCluster<T, d>::cleanup() {
  i_free_aligned(_data);
  _nr_points = 0;
}

template <typename T>
inline int i_assign_point_to_voxel(const T* data, T bound_x_min, T bound_x_max,
                                   T bound_y_min, T bound_y_max, T bound_z_min,
                                   T bound_z_max, T voxel_width_x_rec,
                                   T voxel_width_y_rec, int nr_voxel_x,
                                   int nr_voxel_y) {
  int i, j, k;
  T x = data[0];
  T y = data[1];
  T z = data[2];

  /*points that are outside the defined BBOX are ignored*/
  if (x < bound_x_min || x > bound_x_max || y < bound_y_min ||
      y > bound_y_max || z < bound_z_min || z > bound_z_max) {
    return (-1);
  }

  /*compute the x, y voxel indices*/
  k = i_min(nr_voxel_x - 1, (int)((x - bound_x_min) * voxel_width_x_rec));
  j = i_min(nr_voxel_y - 1, (int)((y - bound_y_min) * voxel_width_y_rec));
  i = (nr_voxel_x * j) + k;
  return (i);
}

template <typename T>
class Voxel {
 public:
  Voxel(){};
  ~Voxel(){};
  Voxel(const Voxel<T>& voxel) {
    _dim_x = voxel._dim_x;
    _dim_y = voxel._dim_y;
    _dim_z = voxel._dim_z;
    _ix = voxel._ix;
    _iy = voxel._iy;
    _iz = voxel._iz;
    i_copy3(voxel._v, _v);
    _indices.assign(voxel._indices.begin(), voxel._indices.end());
  };

  Voxel& operator=(const Voxel<T>& voxel) {
    if (this != &voxel) {
      this->_dim_x = voxel._dim_x;
      this->_dim_y = voxel._dim_y;
      this->_dim_z = voxel._dim_z;
      this->_ix = voxel._ix;
      this->_iy = voxel._iy;
      this->_iz = voxel._iz;
      i_copy3(voxel._v, this->_v);
      this->_indices.assign(voxel._indices.begin(), voxel._indices.end());
    }
    return (*this);
  };

  void init(const T v[3], T dim_x, T dim_y, T dim_z, int ix, int iy, int iz) {
    i_copy3(v, _v);
    _dim_x = dim_x;
    _dim_y = dim_y;
    _dim_z = dim_z;
    _ix = ix;
    _iy = iy;
    _iz = iz;
    _indices.clear();
  }

  void init(T v_x, T v_y, T v_z, T dim_x, T dim_y, T dim_z, int ix, int iy,
            int iz) {
    _v[0] = v_x;
    _v[1] = v_y;
    _v[2] = v_z;
    _dim_x = dim_x;
    _dim_y = dim_y;
    _dim_z = dim_z;
    _ix = ix;
    _iy = iy;
    _iz = iz;
    _indices.clear();
  }

  void reset() {
    i_zero3(_v);
    _dim_x = _dim_y = _dim_z = 0;
    _ix = _iy = _iz = 0;
    _indices.clear();
  }

  void reserve(unsigned int size) { _indices.reserve(size); }

  void push_back(int id) { _indices.push_back(id); }

  unsigned int capacity() const { return (unsigned int)_indices.capacity(); }

  unsigned int nr_points() const { return (unsigned int)_indices.size(); }

  bool empty() const { return _indices.empty(); }

  T _v[3], _dim_x, _dim_y, _dim_z;

  /*voxel indices in the X, Y, Z dimensions - meaningful for voxel grid*/
  int _ix, _iy, _iz;

  /*point indices*/
  std::vector<int> _indices;
};

/*-----Voxel Grid XY-----*/
template <typename T>
class VoxelGridXY {
  /*assuming at most 320000 points*/
  static const unsigned int _nr_max_reserved_points = 320000;

 public:
  VoxelGridXY();
  VoxelGridXY(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
              T spatial_bound_x_min, T spatial_bound_x_max,
              T spatial_bound_y_min, T spatial_bound_y_max,
              T spatial_bound_z_min, T spatial_bound_z_max);

  VoxelGridXY& operator=(const VoxelGridXY<T>& vg) {
    if (this != &vg) {
      this->_initialized = vg.initialized();
      this->_nr_points = vg.nr_points();
      this->_nr_point_element = vg.nr_point_element();
      this->_nr_voxel_x = vg.nr_voxel_x();
      this->_nr_voxel_y = vg.nr_voxel_y();
      this->_nr_voxel_z = vg.nr_voxel_z();
      this->_data = vg.const_data();
      vg.get_grid_dimension(this->_dim_x[0], this->_dim_x[1], this->_dim_y[0],
                            this->_dim_y[1], this->_dim_z[0], this->_dim_z[1]);
      this->_voxels.resize(vg.nr_voxel());
      for (unsigned int i = 0; i < vg.nr_voxel(); ++i) {
        this->_voxels[i] = vg[i];
      }

      this->alloc_aligned_memory();
    }
    return (*this);
  }

  void cleanup();

  ~VoxelGridXY() {
    i_free_aligned<float>(_mem_aligned16_f32);
    i_free_aligned<int>(_mem_aligned16_i32);
    cleanup();
  }

  bool alloc(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
             T spatial_bound_x_min, T spatial_bound_x_max,
             T spatial_bound_y_min, T spatial_bound_y_max,
             T spatial_bound_z_min, T spatial_bound_z_max);

  // non-sse2 version
  bool set(const T* data /*pointer to the point cloud memory*/,
           unsigned int nr_points, unsigned int nr_point_element);

  /*sse2 version: only for float type input data*/
  bool set_s(const float* data /*pointer to the point cloud memory*/,
             unsigned int nr_points, unsigned int nr_point_element);

  bool set(const T* data /*pointer to the point cloud memory*/,
           unsigned int nr_points, unsigned int nr_point_element,
           unsigned int nr_voxel_x, unsigned int nr_voxel_y,
           T spatial_bound_x_min, T spatial_bound_x_max, T spatial_bound_y_min,
           T spatial_bound_y_max, T spatial_bound_z_min, T spatial_bound_z_max,
           bool force_bound = true);

  bool initialized() const { return _initialized; }

  unsigned int nr_voxel() const { return (unsigned int)_voxels.size(); }

  unsigned int nr_voxel_x() const { return _nr_voxel_x; }

  unsigned int nr_voxel_y() const { return _nr_voxel_y; }

  unsigned int nr_voxel_z() const { return (1); }

  unsigned int nr_points() const { return _nr_points; }

  unsigned int nr_point_element() const { return _nr_point_element; }

  unsigned int nr_indexed_points() const;

  std::vector<Voxel<T>>& get_voxels() { return _voxels; }

  const std::vector<Voxel<T>>& get_const_voxels() const { return _voxels; }

  void set_voxels(const std::vector<Voxel<T>>& voxels) {
    _voxels.assign(voxels.begin(), voxels.end());
  }

  bool get_grid_dimension(T& dim_min_x, T& dim_max_x, T& dim_min_y,
                          T& dim_max_y, T& dim_min_z, T& dim_max_z) const {
    if (!_initialized) {
      dim_min_x = dim_max_x = dim_min_y = dim_max_y = dim_min_z = dim_max_z =
          (T)0;
      return (false);
    }
    dim_min_x = _dim_x[0];
    dim_max_x = _dim_x[1];
    dim_min_y = _dim_y[0];
    dim_max_y = _dim_y[1];
    dim_min_z = _dim_z[0];
    dim_max_z = _dim_z[1];
    return (true);
  }

  void set_grid_dimension(T dim_min_x, T dim_max_x, T dim_min_y, T dim_max_y,
                          T dim_min_z, T dim_max_z) {
    _dim_x[0] = dim_min_x;
    _dim_x[1] = dim_max_x;
    _dim_y[0] = dim_min_y;
    _dim_y[1] = dim_max_y;
    _dim_z[0] = dim_min_z;
    _dim_z[1] = dim_max_z;
  }

  bool get_voxel_dimension(T& voxel_width_x, T& voxel_width_y,
                           T& voxel_width_z) const {
    if (!_initialized) {
      voxel_width_x = voxel_width_y = voxel_width_z = (T)0;
      return (false);
    }
    voxel_width_x = _voxel_dim[0];
    voxel_width_y = _voxel_dim[1];
    voxel_width_z = _voxel_dim[2];
    return (true);
  }

  void set_voxel_dimension(T voxel_width_x, T voxel_width_y, T voxel_width_z) {
    _voxel_dim[0] = voxel_width_x;
    _voxel_dim[1] = voxel_width_y;
    _voxel_dim[2] = voxel_width_z;
  }

  void set_nr_points(unsigned int nr_points) { _nr_points = nr_points; }

  void set_nr_point_element(unsigned int nr_point_element) {
    _nr_point_element = nr_point_element;
  }

  void set_nr_voxel(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
                    unsigned int nr_voxel_z) {
    _nr_voxel_x = nr_voxel_x;
    _nr_voxel_y = nr_voxel_y;
    _nr_voxel_z = nr_voxel_z;
  }

  void set_point_clouds_data(const T* data) { _data = data; }

  void set_initialized(bool initialized) { _initialized = initialized; }

  int get_all_indices(std::vector<int>& indices) const;

  int which_voxel(T x, T y, T z) const;

  /* add by Fangzhen Li on 03/27/17 */
  bool get_voxel_coordinate_xy(T x, T y, int* row, int* col) const;

  Voxel<T>& operator[](unsigned int i) {
    assert(i >= 0 && i < _voxels.size());
    return (_voxels[i]);
  }

  const Voxel<T>& operator[](unsigned int i) const {
    assert(i >= 0 && i < _voxels.size());
    return (_voxels[i]);
  }

  Voxel<T>& operator[](int i) {
    assert(i >= 0 && i < (int)_voxels.size());
    return (_voxels[i]);
  }
  const Voxel<T>& operator[](int i) const {
    assert(i >= 0 && i < (int)_voxels.size());
    return (_voxels[i]);
  }

  Voxel<T>& operator()(unsigned int iy, unsigned int ix) {
    assert(iy < _nr_voxel_y && ix < _nr_voxel_x);
    unsigned int i = _nr_voxel_x * iy + ix;
    assert(i < _voxels.size());
    return (_voxels[i]);
  }

  const Voxel<T>& operator()(unsigned int iy, unsigned int ix) const {
    assert(iy < _nr_voxel_y && ix < _nr_voxel_x);
    unsigned int i = _nr_voxel_x * iy + ix;
    assert(i < _voxels.size());
    return (_voxels[i]);
  }

  T* data() { return (_data); }

  const T* const_data() const { return (_data); }

 private:
  void reserve();
  bool alloc_aligned_memory();

 private:
  unsigned int _nr_points, _nr_point_element;
  unsigned int _nr_voxel_x, _nr_voxel_y, _nr_voxel_z;
  const T* _data; /*point clouds memory*/
  bool _initialized;
  T _dim_x[2], _dim_y[2], _dim_z[2], _voxel_dim[3];
  float* _mem_aligned16_f32;
  int* _mem_aligned16_i32;
  std::vector<Voxel<T>> _voxels;
};

template <typename T>
VoxelGridXY<T>::VoxelGridXY()
    : _data(NULL),
      _mem_aligned16_f32(NULL),
      _mem_aligned16_i32(NULL),
      _nr_points(0),
      _nr_point_element(0),
      _nr_voxel_x(0),
      _nr_voxel_y(0),
      _nr_voxel_z(1),
      _initialized(false) {
  i_zero2(_dim_x);
  i_zero2(_dim_y);
  i_zero2(_dim_z);
  i_zero3(_voxel_dim);
  alloc_aligned_memory();
};

template <typename T>
void VoxelGridXY<T>::cleanup() {
  _initialized = false;
  _data = NULL;
  _nr_points = _nr_point_element = _nr_voxel_x = _nr_voxel_y = 0;
  _nr_voxel_z = 1;
  i_zero2(_dim_x);
  i_zero2(_dim_y);
  i_zero2(_dim_z);
  i_zero3(_voxel_dim);
  _voxels.clear();
}

template <typename T>
VoxelGridXY<T>::VoxelGridXY(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
                            T spatial_bound_x_min, T spatial_bound_x_max,
                            T spatial_bound_y_min, T spatial_bound_y_max,
                            T spatial_bound_z_min, T spatial_bound_z_max) {
  alloc(nr_voxel_x, nr_voxel_y, spatial_bound_x_min, spatial_bound_x_max,
        spatial_bound_y_min, spatial_bound_y_max, spatial_bound_z_min,
        spatial_bound_z_max);
  alloc_aligned_memory();
  _initialized = false;
}

template <typename T>
void VoxelGridXY<T>::reserve() {
  int r, c, i = 0, m = 0;
  unsigned int n = (_nr_voxel_x * _nr_voxel_y);

  if (n == 0 || _voxels.size() != n) {
    return;
  }

  double cen_x = ((double)(_nr_voxel_x - 1)) * 0.5;
  double cen_y = ((double)(_nr_voxel_y - 1)) * 0.5;

  /*normalization factor = 1/(2sigma*sigma)*/
  /*hardcode sigma 10.0*/
  double sigma = (double)i_average(_nr_voxel_x, _nr_voxel_y) / 10.0;
  double nf = i_div(0.5, (sigma * sigma));
  double dr, drsqr, dc, dcsqr, v, ksum = 0.0;
  std::vector<double> kernel(n, 0.0);

  /*pre-compute kernel*/
  for (r = 0; r < (int)_nr_voxel_y; ++r) {
    dr = (double)r - cen_y;
    drsqr = i_sqr(dr);
    for (c = 0; c < (int)_nr_voxel_x; ++c) {
      dc = (double)c - cen_x;
      dcsqr = i_sqr(dc);
      v = i_exp(-(drsqr + dcsqr) * nf);
      ksum += v;
      kernel[i++] = v;
    }
  }

  /*normalize the kernel, sum to 1.0*/
  v = i_rec(ksum);
  for (i = 0; i < (int)n; ++i) {
    kernel[i] *= v;
  }

  /*alloc at least 8 positions*/
  for (i = 0; i < (int)n; ++i) {
    m = i_max(8, (int)(_nr_max_reserved_points * kernel[i]));
    _voxels[i].reserve(m);
  }

  return;
}

template <typename T>
bool VoxelGridXY<T>::alloc_aligned_memory() {
  if (!_mem_aligned16_f32) {
    _mem_aligned16_f32 = i_alloc_aligned<float>(4, 4);
  }
  if (!_mem_aligned16_i32) {
    _mem_aligned16_i32 = i_alloc_aligned<int>(4, 4);
  }
  return (_mem_aligned16_f32 != NULL && _mem_aligned16_i32 != NULL &&
          i_verify_alignment(_mem_aligned16_f32, 4) &&
          i_verify_alignment(_mem_aligned16_i32, 4));
}

template <typename T>
int VoxelGridXY<T>::which_voxel(T x, T y, T z) const {
  int j, k;
  if (!_initialized) {
    return (-1);
  }

  if (x < _dim_x[0] || x > _dim_x[1] || y < _dim_y[0] || y > _dim_y[1] ||
      z < _dim_z[0] || z > _dim_z[1]) {
    return (-1); /*points that are outside the defined BBOX are ignored*/
  }

  k = (int)i_max(
      (unsigned int)0,
      i_min(_nr_voxel_x - 1, (unsigned int)((x - _dim_x[0]) / _voxel_dim[0])));
  j = (int)i_max(
      (unsigned int)0,
      i_min(_nr_voxel_y - 1, (unsigned int)((y - _dim_y[0]) / _voxel_dim[1])));
  return (_nr_voxel_x * j + k);
}

/* add by Fangzhen Li on 03/27/17 */
template <typename T>
bool VoxelGridXY<T>::get_voxel_coordinate_xy(T x, T y, int* row,
                                             int* col) const {
  if (x < _dim_x[0] || x > _dim_x[1] || y < _dim_y[0] || y > _dim_y[1]) {
    return false; /*points that are outside the defined BBOX are ignored*/
  }

  *col = (int)((x - _dim_x[0]) / _voxel_dim[0]);
  *row = (int)((y - _dim_y[0]) / _voxel_dim[1]);
  return true;
}

template <typename T>
bool VoxelGridXY<T>::alloc(unsigned int nr_voxel_x, unsigned int nr_voxel_y,
                           T spatial_bound_x_min, T spatial_bound_x_max,
                           T spatial_bound_y_min, T spatial_bound_y_max,
                           T spatial_bound_z_min, T spatial_bound_z_max) {
  if (!nr_voxel_x || !nr_voxel_y) {
    _initialized = false;
    return _initialized;
  }
  _nr_voxel_x = nr_voxel_x;
  _nr_voxel_y = nr_voxel_y;

  unsigned int i, j, k, n = 0;
  unsigned int nr_voxel_xy = (_nr_voxel_x * _nr_voxel_y);
  unsigned int nr_voxel = nr_voxel_xy;

  T vx, vy, vz, voxel_width_x, voxel_width_y, voxel_width_z;

  _voxels.clear();
  _voxels.resize(nr_voxel);

  /*voxel grid dimesion is forced to be the manual input*/
  _dim_x[0] = spatial_bound_x_min;
  _dim_x[1] = spatial_bound_x_max;

  _dim_y[0] = spatial_bound_y_min;
  _dim_y[1] = spatial_bound_y_max;

  _dim_z[0] = spatial_bound_z_min;
  _dim_z[1] = spatial_bound_z_max;

  T span_x = (_dim_x[1] - _dim_x[0]);
  T span_y = (_dim_y[1] - _dim_y[0]);
  T span_z = (_dim_z[1] - _dim_z[0]);

  assert(span_x > 0 && span_y > 0 && span_z > 0);

  _voxel_dim[0] = voxel_width_x = i_div(span_x, (int)_nr_voxel_x);
  _voxel_dim[1] = voxel_width_y = i_div(span_y, (int)_nr_voxel_y);
  _voxel_dim[2] = voxel_width_z = span_z;

  std::vector<T> vxs(_nr_voxel_x);

  vxs[0] = _dim_x[0];
  for (i = 1; i < _nr_voxel_x; i++) {
    vxs[i] = vxs[i - 1] + voxel_width_x;
  }

  vy = _dim_y[0];
  vz = _dim_z[0];

  for (j = 0; j < _nr_voxel_y; j++) {
    for (k = 0; k < _nr_voxel_x; k++) {
      vx = vxs[k];
      _voxels[n++].init(vx, vy, vz, voxel_width_x, voxel_width_y, voxel_width_z,
                        (int)k, (int)j, 0);
    }
    vy += voxel_width_y;
  }

  /*pre-alloc capacaity for _indices vector - speed-up the code slightly*/
  reserve();
  return (true);
}

template <typename T>
bool VoxelGridXY<T>::set(const T* data, unsigned int nr_points,
                         unsigned int nr_point_element) {
  if (!data || !nr_points || !nr_point_element || !_nr_voxel_x ||
      !_nr_voxel_y || _nr_voxel_z != 1) {
    _initialized = false;
    return _initialized;
  }

  _data = data;
  _nr_points = nr_points;
  _nr_point_element = nr_point_element;

  unsigned int nr_voxel = (_nr_voxel_x * _nr_voxel_y);
  unsigned int i, nd = 0;
  int id, n = 0;

  T voxel_width_x_rec = i_rec(_voxel_dim[0]);
  T voxel_width_y_rec = i_rec(_voxel_dim[1]);

  T bound_x_min = _dim_x[0];
  T bound_x_max = _dim_x[1];
  T bound_y_min = _dim_y[0];
  T bound_y_max = _dim_y[1];
  T bound_z_min = _dim_z[0];
  T bound_z_max = _dim_z[1];

  /*clear the old index buffer*/
  for (i = 0; i < nr_voxel; ++i) {
    _voxels[i]._indices.clear();
  }

  /*assign point indices*/
  for (; n < (int)_nr_points; n++, nd += _nr_point_element) {
    id = i_assign_point_to_voxel(
        _data + nd, bound_x_min, bound_x_max, bound_y_min, bound_y_max,
        bound_z_min, bound_z_max, voxel_width_x_rec, voxel_width_y_rec,
        (int)_nr_voxel_x, (int)_nr_voxel_y);

    if (id >= 0) {
      _voxels[id].push_back(n);
    }
  }

  _initialized = true;
  return (_initialized);
}

template <typename T>
bool VoxelGridXY<T>::set_s(
    const float* data /*pointer to the point cloud memory*/,
    unsigned int nr_points, unsigned int nr_point_element) {
  if (!data || !nr_points || !nr_point_element || !_nr_voxel_x ||
      !_nr_voxel_y || _nr_voxel_z != 1) {
    _initialized = false;
    return _initialized;
  }

  _data = data;
  _nr_points = nr_points;
  _nr_point_element = nr_point_element;

  unsigned int nr_voxel = (_nr_voxel_x * _nr_voxel_y);
  unsigned int i, n = 0;
  int id;

  float voxel_width_x_rec = i_rec((float)_voxel_dim[0]);
  float voxel_width_y_rec = i_rec((float)_voxel_dim[1]);

  __m128i iv_nr_voxel_x =
      _mm_setr_epi32((int)_nr_voxel_x, 0, (int)_nr_voxel_x, 0);
  __m128i iv_nr_voxel_x_m1 = _mm_set1_epi32((int)(_nr_voxel_x - 1));
  __m128i iv_nr_voxel_y_m1 = _mm_set1_epi32((int)(_nr_voxel_y - 1));

  __m128 v_width_x_rec = _mm_set_ps1(voxel_width_x_rec);
  __m128 v_width_y_rec = _mm_set_ps1(voxel_width_y_rec);

  __m128 v_x_min = _mm_set_ps1(_dim_x[0]);
  __m128 v_x_max = _mm_set_ps1(_dim_x[1]);
  __m128 v_y_min = _mm_set_ps1(_dim_y[0]);
  __m128 v_y_max = _mm_set_ps1(_dim_y[1]);
  __m128 v_z_min = _mm_set_ps1(_dim_z[0]);
  __m128 v_z_max = _mm_set_ps1(_dim_z[1]);

  __m128 v_cmp_x, v_cmp_y, v_cmp_z, v_in_roi;
  v_in_roi = _mm_setr_ps(1.0, 1.0, 1.0, 1.0);

  __m128 v_xs, v_ys, v_zs;
  __m128i iv_indices, iv_x_indices, iv_y_indices, iv_v_indices_02,
      iv_v_indices_13;

  for (i = 0; i < nr_voxel; ++i) {
    _voxels[i]._indices.clear();
  }

  unsigned int nr_loops = (nr_points >> 2);
  unsigned int nr_fast_processed = (nr_loops << 2);
  unsigned int remainder = nr_points - nr_fast_processed;

  unsigned int d1 = nr_point_element;
  unsigned int d2 = (nr_point_element << 1);
  unsigned int d3 = d1 + d2;
  unsigned int d4 = (nr_point_element << 2);

  /*xyz are required to be stored in continuous memory*/
  const float* cptr_x = data;
  const float* cptr_y = data + 1;
  const float* cptr_z = data + 2;
  const float* cptr_remainder = data + (nr_fast_processed * nr_point_element);

  for (i = 0; i < nr_loops; ++i, n += 4) {
    /*set memory*/
    v_xs = _mm_setr_ps(cptr_x[0], cptr_x[d1], cptr_x[d2], cptr_x[d3]);
    v_ys = _mm_setr_ps(cptr_y[0], cptr_y[d1], cptr_y[d2], cptr_y[d3]);
    v_zs = _mm_setr_ps(cptr_z[0], cptr_z[d1], cptr_z[d2], cptr_z[d3]);

    /*compare range:*/
    v_cmp_x =
        _mm_and_ps(_mm_cmpge_ps(v_xs, v_x_min), _mm_cmple_ps(v_xs, v_x_max));
    v_cmp_y =
        _mm_and_ps(_mm_cmpge_ps(v_ys, v_y_min), _mm_cmple_ps(v_ys, v_y_max));
    v_cmp_z =
        _mm_and_ps(_mm_cmpge_ps(v_zs, v_z_min), _mm_cmple_ps(v_zs, v_z_max));
    v_in_roi = _mm_and_ps(_mm_and_ps(v_cmp_x, v_cmp_y), v_cmp_z);

    /*vector operations, cast into signed integers*/
    v_xs = _mm_sub_ps(v_xs, v_x_min);
    v_xs = _mm_mul_ps(v_xs, v_width_x_rec);
    iv_x_indices = _mm_cvttps_epi32(v_xs); /*truncate towards zero*/
    iv_x_indices = _mm_min_epi32(iv_nr_voxel_x_m1, iv_x_indices);

    /*vector operations, cast into signed integers*/
    v_ys = _mm_sub_ps(v_ys, v_y_min);
    v_ys = _mm_mul_ps(v_ys, v_width_y_rec);
    iv_y_indices = _mm_cvttps_epi32(v_ys); /*truncate towards zero*/
    iv_y_indices = _mm_min_epi32(iv_nr_voxel_y_m1, iv_y_indices);

    iv_v_indices_02 = _mm_mullo_epi32(iv_y_indices, iv_nr_voxel_x);
    iv_y_indices = _mm_shuffle_epi32(iv_y_indices, _MM_SHUFFLE(2, 3, 0, 1));

    iv_v_indices_13 = _mm_mullo_epi32(iv_y_indices, iv_nr_voxel_x);
    iv_v_indices_13 =
        _mm_shuffle_epi32(iv_v_indices_13, _MM_SHUFFLE(2, 3, 0, 1));

    iv_indices = _mm_add_epi32(iv_v_indices_02, iv_v_indices_13);
    iv_indices = _mm_add_epi32(iv_indices, iv_x_indices);

    /*store values from registers to memory*/
    /*address 16byte-aligned*/
    _mm_store_ps(_mem_aligned16_f32, v_in_roi);
    /*address 16byte-aligned*/
    _mm_store_si128((__m128i*)_mem_aligned16_i32, iv_indices);

    if (_mem_aligned16_f32[0] != 0) {
      _voxels[_mem_aligned16_i32[0]]._indices.push_back(n);
    }
    if (_mem_aligned16_f32[1] != 0) {
      _voxels[_mem_aligned16_i32[1]]._indices.push_back(n + 1);
    }
    if (_mem_aligned16_f32[2] != 0) {
      _voxels[_mem_aligned16_i32[2]]._indices.push_back(n + 2);
    }
    if (_mem_aligned16_f32[3] != 0) {
      _voxels[_mem_aligned16_i32[3]]._indices.push_back(n + 3);
    }
    cptr_x += d4;
    cptr_y += d4;
    cptr_z += d4;
  }

  /*handling remaining points*/
  for (i = 0; i < remainder; i++, n++) {
    id = i_assign_point_to_voxel(cptr_remainder, _dim_x[0], _dim_x[1],
                                 _dim_y[0], _dim_y[1], _dim_z[0], _dim_z[1],
                                 voxel_width_x_rec, voxel_width_y_rec,
                                 (int)_nr_voxel_x, (int)_nr_voxel_y);

    if (id >= 0) {
      _voxels[id]._indices.push_back(n);
    }

    cptr_remainder += nr_point_element;
  }

  _initialized = true;
  return (_initialized);
}

template <typename T>
bool VoxelGridXY<T>::set(const T* data, unsigned int nr_points,
                         unsigned int nr_point_element, unsigned int nr_voxel_x,
                         unsigned int nr_voxel_y, T spatial_bound_x_min,
                         T spatial_bound_x_max, T spatial_bound_y_min,
                         T spatial_bound_y_max, T spatial_bound_z_min,
                         T spatial_bound_z_max, bool force_bound) {
  if (!data || !nr_points || !nr_point_element || !nr_voxel_x || !nr_voxel_y) {
    _initialized = false;
    return _initialized;
  }

  _data = data;
  _nr_points = nr_points;
  _nr_point_element = nr_point_element;
  _nr_voxel_x = nr_voxel_x;
  _nr_voxel_y = nr_voxel_y;
  _nr_voxel_z = 1;

  unsigned int i, j, k, n = 0, nd = 0;
  unsigned int nr_voxel_xy = (_nr_voxel_x * _nr_voxel_y);
  unsigned int nr_voxel = nr_voxel_xy;
  unsigned int nr_voxel_xm1 = nr_voxel_x - 1;
  unsigned int nr_voxel_ym1 = nr_voxel_y - 1;
  T vx, vy, vz, x, y, z;
  T voxel_width_x, voxel_width_y, voxel_width_z;

  _voxels.clear();
  _voxels.resize(nr_voxel);

  /*if force_bound then the voxel grid dimesion is forced to be the manual
   * input*/
  if (force_bound) {
    _dim_x[0] = spatial_bound_x_min;
    _dim_x[1] = spatial_bound_x_max;

    _dim_y[0] = spatial_bound_y_min;
    _dim_y[1] = spatial_bound_y_max;

    _dim_z[0] = spatial_bound_z_min;
    _dim_z[1] = spatial_bound_z_max;
  } else {
    i_get_pointclouds_dim_w_bound(
        _data, _nr_points, 0, _nr_point_element, _dim_x[0], _dim_x[1],
        _dim_y[0], _dim_y[1], _dim_z[0], _dim_z[1], spatial_bound_x_min,
        spatial_bound_x_max, spatial_bound_y_min, spatial_bound_y_max,
        spatial_bound_z_min, spatial_bound_z_max);
  }

  T dim_x_min = _dim_x[0];
  T dim_x_max = _dim_x[1];

  T dim_y_min = _dim_y[0];
  T dim_y_max = _dim_y[1];

  T dim_z_min = _dim_z[0];
  T dim_z_max = _dim_z[1];

  T span_x = (dim_x_max - dim_x_min);
  T span_y = (dim_y_max - dim_y_min);
  T span_z = (dim_z_max - dim_z_min);

  assert(span_x > 0 && span_y > 0 && span_z > 0);

  _voxel_dim[0] = voxel_width_x = i_div(span_x, (int)_nr_voxel_x);
  _voxel_dim[1] = voxel_width_y = i_div(span_y, (int)_nr_voxel_y);
  _voxel_dim[2] = voxel_width_z = span_z;

  T voxel_width_x_rec = i_rec(_voxel_dim[0]);
  T voxel_width_y_rec = i_rec(_voxel_dim[1]);

  std::vector<T> vxs(_nr_voxel_x);
  vxs[0] = _dim_x[0];

  for (i = 1; i < _nr_voxel_x; i++) {
    vxs[i] = vxs[i - 1] + voxel_width_x;
  }

  vy = _dim_y[0];
  vz = _dim_z[0];

  for (j = 0; j < _nr_voxel_y; j++) {
    for (k = 0; k < _nr_voxel_x; k++) {
      vx = vxs[k];
      _voxels[n++].init(vx, vy, vz, voxel_width_x, voxel_width_y, voxel_width_z,
                        (int)k, (int)j, 0);
    }
    vy += voxel_width_y;
  }

  /*assign point indices*/
  for (; n < _nr_points; n++, nd += _nr_point_element) {
    x = _data[nd];
    y = _data[nd + 1];
    z = _data[nd + 2];

    if (x < dim_x_min || x > dim_x_max || y < dim_y_min || y > dim_y_max ||
        z < dim_z_min || z > dim_z_max) {
      /*points that are outside the defined BBOX are ignored*/
      continue;
    }

    k = i_max((unsigned int)0,
              i_min(nr_voxel_xm1,
                    (unsigned int)((x - dim_x_min) * voxel_width_x_rec)));
    j = i_max((unsigned int)0,
              i_min(nr_voxel_ym1,
                    (unsigned int)((y - dim_y_min) * voxel_width_y_rec)));
    i = nr_voxel_x * j + k;

    assert(i < nr_voxel);
    _voxels[i]._indices.push_back(n);
  }
  _initialized = true;
  return _initialized;
}

template <typename T>
int VoxelGridXY<T>::get_all_indices(std::vector<int>& indices) const {
  unsigned int i, j;
  indices.clear();

  if (!_initialized) {
    return -1;
  }
  for (i = 0; i < _voxels.size(); ++i) {
    for (j = 0; j < _voxels[i]._indices.size(); ++j) {
      indices.push_back(_voxels[i]._indices.at(j));
    }
  }

  return (int)indices.size();
}

template <typename T>
unsigned int VoxelGridXY<T>::nr_indexed_points() const {
  if (!_initialized) {
    return 0;
  }
  unsigned int i, n = 0;
  for (i = 0; i < _voxels.size(); ++i) {
    n += _voxels[i]._indices.size();
  }
  return n;
}

template <typename T>
static void i_push_back_vectors(const std::vector<T>& src,
                                std::vector<T>& dst) {
  unsigned int i = src.size();
  unsigned int j = dst.size();

  if (i == 0) {
    return;
  } else if (j == 0) {
    dst.assign(src.begin(), src.end());
  } else {
    dst.resize(i + j);
    i_copy(src.data(), dst.data() + j, (int)i);
  }
  return;
}

/*VoxelgridXY downsample functions*/
template <typename T>
bool i_downsample_voxelgridxy(const VoxelGridXY<T>& src, VoxelGridXY<T>& dst,
                              unsigned int dsf_dim_x, unsigned int dsf_dim_y) {
  if (!src.initialized()) {
    return (false);
  } else if (dsf_dim_x == 0 && dsf_dim_y == 0) {
    dst = src;
    return (true);
  } else {
    if (&src == &dst) {
      return (false); /*not allowed to downsample a voxel grid from itself*/
    }
  }

  dst.cleanup(); /*perform cleanup first*/

  /*# of voxels of the src voxel grid*/
  unsigned int nr_voxel_x_src = src.nr_voxel_x();
  unsigned int nr_voxel_y_src = src.nr_voxel_y();
  unsigned int nr_voxel_z_src = src.nr_voxel_z();

  assert(nr_voxel_z_src == 1);

  /*scale factors*/
  unsigned int sf_x = (unsigned int)i_pow((unsigned int)2, dsf_dim_x);
  unsigned int sf_y = (unsigned int)i_pow((unsigned int)2, dsf_dim_y);

  /*compute the # of voxels for the new scale*/
  unsigned int nr_voxel_x_dst = nr_voxel_x_src / sf_x;
  unsigned int nr_voxel_y_dst = nr_voxel_y_src / sf_y;
  unsigned int nr_voxel_z_dst = nr_voxel_z_src;

  if (!nr_voxel_x_dst || !nr_voxel_y_dst || !nr_voxel_z_dst) {
    return (false); /*do not continue*/
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

  /*the dimension of the voxels in the src voxel grid*/
  src.get_voxel_dimension(voxel_width_x_src, voxel_width_y_src,
                          voxel_width_z_src);

  src.get_grid_dimension(dim_min_x_src, dim_max_x_src, dim_min_y_src,
                         dim_max_y_src, dim_min_z_src, dim_max_z_src);

  /*new voxel dimensions after downsampling the 3D grid*/
  T voxel_width_x_dst = (sf_x * voxel_width_x_src);
  T voxel_width_y_dst = (sf_y * voxel_width_y_src);
  T voxel_width_z_dst = voxel_width_z_src;

  /*new grid dimensions after downsampling the 3D grid*/
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

  /*set dst*/
  dst.set_nr_points(src.nr_points());
  dst.set_nr_point_element(src.nr_point_element());
  dst.set_voxel_dimension(voxel_width_x_dst, voxel_width_y_dst,
                          voxel_width_z_dst);
  dst.set_grid_dimension(dim_x_dst[0], dim_x_dst[1], dim_y_dst[0], dim_y_dst[1],
                         dim_z_dst[0], dim_z_dst[1]);
  dst.set_nr_voxel(nr_voxel_x_dst, nr_voxel_y_dst, nr_voxel_z_dst);
  dst.set_point_clouds_data(src.const_data());

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
                     voxel_width_z_dst, (int)c, (int)r, (int)0);
      /*collect samples from its lower scale:*/
      for (i = 0; i < sf_y; ++i) {
        rspi = rs + i;
        for (j = 0; j < sf_x; ++j) {
          i_push_back_vectors(src(rspi, cs + j)._indices, voxels[n]._indices);
        }
      }
      /*increase voxel index by 1*/
      n++;
    }
  }
  dst.set_voxels(voxels);
  dst.set_initialized(true);
  return (true);
}

/*-----Multiscale Voxel Grid Pyramid-----*/
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
    alloc(nr_scale, nr_voxel_x_base, nr_voxel_y_base, dsf_x, dsf_y,
          spatial_bound_x_min, spatial_bound_x_max, spatial_bound_y_min,
          spatial_bound_y_max, spatial_bound_z_min, spatial_bound_z_max);
  };

  void cleanup();

  ~VoxelGridXYPyramid() { cleanup(); }

  bool alloc(unsigned int nr_scale, unsigned int nr_voxel_x_base,
             unsigned int nr_voxel_y_base, unsigned int dsf_x,
             unsigned int dsf_y, DATA_TYPE spatial_bound_x_min,
             DATA_TYPE spatial_bound_x_max, DATA_TYPE spatial_bound_y_min,
             DATA_TYPE spatial_bound_y_max, DATA_TYPE spatial_bound_z_min,
             DATA_TYPE spatial_bound_z_max);

  /*non-sse2 version*/
  bool set(const DATA_TYPE* pc /*pointer to the point cloud*/,
           /*total number of points of the point cloud - base level*/
           unsigned int nr_points, unsigned int nr_point_element);

  /*the sse2 version - speed up the voxel grid construction, for float type
   * points only*/
  bool set_s(const float* pc /*pointer to the point cloud*/,
             /*total number of points of the point cloud - base level*/
             unsigned int nr_points, unsigned int nr_point_element);

  bool set(const DATA_TYPE* pc /*pointer to the point cloud*/,
           unsigned int nr_scale,
           /*total number of points of the point cloud - base level*/
           unsigned int nr_points, unsigned int nr_point_element,
           unsigned int nr_voxel_x_base, unsigned int nr_voxel_y_base,
           unsigned int dsf_x, unsigned int dsf_y,
           DATA_TYPE spatial_bound_x_min, DATA_TYPE spatial_bound_x_max,
           DATA_TYPE spatial_bound_y_min, DATA_TYPE spatial_bound_y_max,
           DATA_TYPE spatial_bound_z_min, DATA_TYPE spatial_bound_z_max);

  unsigned int nr_scale() const { return (unsigned int)_vgrids.size(); }

  unsigned int nr_voxel(unsigned int i = 0) const {
    return (i < _vgrids.size()) ? _vgrids[i].nr_voxel() : 0;
  }

  unsigned int nr_voxel_x(unsigned int i = 0) const {
    return (i < _vgrids.size()) ? _vgrids[i].nr_voxel_x() : 0;
  }

  unsigned int nr_voxel_y(unsigned int i = 0) const {
    return (i < _vgrids.size()) ? _vgrids[i].nr_voxel_y() : 0;
  }

  unsigned int nr_voxel_z(unsigned int i = 0) const {
    return (i < _vgrids.size()) ? _vgrids[i].nr_voxel_z() : 0;
  }

  unsigned int nr_points(unsigned int scale) const {
    return (scale < _vgrids.size()) ? _vgrids[scale].nr_points() : 0;
  }

  unsigned int nr_points() const { return _nr_points; }

  unsigned int nr_point_element() const { return _nr_point_element; }

  bool initialized() const;

  unsigned int get_dsf_x() const { return _dsf_x; }
  unsigned int get_dsf_y() const { return _dsf_y; }
  unsigned int get_dsf_z() const { return (0); }

  VoxelGridXY<DATA_TYPE>& operator[](unsigned int i) {
    assert(i >= 0 && i < _vgrids.size());
    return _vgrids[i];
  }

  const VoxelGridXY<DATA_TYPE>& operator[](unsigned int i) const {
    assert(i >= 0 && i < _vgrids.size());
    return _vgrids[i];
  }

  VoxelGridXY<DATA_TYPE>& operator[](int i) {
    assert(i >= 0 && i < (int)_vgrids.size());
    return _vgrids[i];
  }

  const VoxelGridXY<DATA_TYPE>& operator[](int i) const {
    assert(i >= 0 && i < (int)_vgrids.size());
    return _vgrids[i];
  }

  const DATA_TYPE* const_data() const { return _pc; }

  DATA_TYPE* data() { return _pc; }

 private:
  unsigned int _nr_points, _nr_point_element;
  unsigned int _dsf_x, _dsf_y, _dsf_z;
  const DATA_TYPE* _pc;
  std::vector<VoxelGridXY<DATA_TYPE>> _vgrids;
};

template <typename DATA_TYPE>
VoxelGridXYPyramid<DATA_TYPE>::VoxelGridXYPyramid()
    : _pc(NULL),
      _nr_points(0),
      _nr_point_element(0),
      _dsf_x(0),
      _dsf_y(0),
      _dsf_z(0){};

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::alloc(
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
  unsigned int sf_x = (unsigned int)i_pow((unsigned int)2, dsf_x);
  unsigned int sf_y = (unsigned int)i_pow((unsigned int)2, dsf_y);

  _dsf_x = dsf_x;
  _dsf_y = dsf_y;
  _dsf_z = 0;

  _vgrids.clear();
  _vgrids.resize(nr_scale);

  for (scale = 0; scale < nr_scale; ++scale) {
    if (!_vgrids[scale].alloc(nr_voxel_x, nr_voxel_y, spatial_bound_x_min,
                              spatial_bound_x_max, spatial_bound_y_min,
                              spatial_bound_y_max, spatial_bound_z_min,
                              spatial_bound_z_max)) {
      break;
    }
    nr_voxel_x /= sf_x;
    nr_voxel_y /= sf_y;
  }

  if (scale == 0) {
    _vgrids.clear();
    return false;
  } else {
    if (scale != nr_scale) {
      _vgrids.resize(scale);
    }
  }

  return (true);
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::set(const DATA_TYPE* pc,
                                        unsigned int nr_points,
                                        unsigned int nr_point_element) {
  if (!pc || !nr_points || !nr_point_element || _vgrids.empty()) {
    return false;
  }

  unsigned int scale, nr_scale = (unsigned int)_vgrids.size();

  _pc = pc;
  _nr_points = nr_points;
  _nr_point_element = nr_point_element;

  for (scale = 0; scale < nr_scale; ++scale) {
    if (!_vgrids[scale].set(pc, nr_points, nr_point_element)) {
      break;
    }
  }

  /*break in the middle - last (nr_scale - s) grids are invalid*/
  if (scale < nr_scale) {
    _vgrids.resize(scale);
  }

  return (_vgrids.size() == nr_scale);
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::set_s(const float* pc,
                                          unsigned int nr_points,
                                          unsigned int nr_point_element) {
  if (!pc || !nr_points || !nr_point_element || _vgrids.empty()) {
    return false;
  }

  unsigned int scale, nr_scale = (unsigned int)_vgrids.size();

  _pc = pc;
  _nr_points = nr_points;
  _nr_point_element = nr_point_element;

  for (scale = 0; scale < nr_scale; ++scale) {
    if (!_vgrids[scale].set_s(pc, nr_points, nr_point_element)) {
      break;
    }
  }

  /*break in the middle - last (nr_scale - s) grids are invalid*/
  if (scale < nr_scale) {
    _vgrids.resize(scale);
  }

  return (_vgrids.size() == nr_scale);
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::set(
    const DATA_TYPE* pc, unsigned int nr_scale, unsigned int nr_points,
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
  _pc = pc;
  _nr_points = nr_points;
  _nr_point_element = nr_point_element;

  _dsf_x = dsf_x;
  _dsf_y = dsf_y;
  _dsf_z = 0;

  _vgrids.clear();
  _vgrids.resize(nr_scale);

  /*set the base level pyramid*/
  if (!_vgrids[0].set(_pc, nr_points, nr_point_element, nr_voxel_x_base,
                      nr_voxel_y_base, spatial_bound_x_min, spatial_bound_x_max,
                      spatial_bound_y_min, spatial_bound_y_max,
                      spatial_bound_z_min, spatial_bound_z_max)) {
    _vgrids.clear();
    return false;
  }

  for (s = 1; s < nr_scale; ++s) {
    if (!i_downsample_voxelgridxy(_vgrids[s - 1], _vgrids[s], _dsf_x, _dsf_y)) {
      break;
    }
  }

  if (s <
      nr_scale) /*break in the middle - last (nr_scale - s) grids are invalid*/
  {
    for (i = 0; i < (nr_scale - s); ++i) {
      _vgrids.pop_back();
    }
  }

  return (_vgrids.size() == nr_scale); /*check if all scales are valid*/
}

template <typename DATA_TYPE>
void VoxelGridXYPyramid<DATA_TYPE>::cleanup() {
  _pc = NULL;
  _vgrids.clear();
  _nr_points = _nr_point_element = _dsf_x = _dsf_y = _dsf_z = 0;
}

template <typename DATA_TYPE>
bool VoxelGridXYPyramid<DATA_TYPE>::initialized() const {
  unsigned int i;
  if (_vgrids.empty()) {
    return false;
  }
  for (i = 0; i < _vgrids.size(); ++i) {
    if (!_vgrids[i].initialized()) {
      break;
    }
  }
  return (i == _vgrids.size());
}

} /*namespace idl*/

#endif
