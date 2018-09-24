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
#ifndef I_LIB_PC_I_UTIL_H
#define I_LIB_PC_I_UTIL_H

#include "../core/i_blas.h"

namespace idl {
template <typename T>
inline void i_get_pointclouds_dim(const T *threeds, int n, T &dim_min_x,
                                  T &dim_max_x, T &dim_min_y, T &dim_max_y,
                                  T &dim_min_z, T &dim_max_z) {
  int i, n3 = n * 3;
  float x, y, z;
  dim_min_x = dim_min_y = dim_min_z = Constant<T>::MAX_VAL() / 2;
  dim_max_x = dim_max_y = dim_max_z = -(Constant<T>::MAX_VAL() / 2);
  for (i = 0; i < n3;) {
    x = threeds[i++];
    y = threeds[i++];
    z = threeds[i++];
    dim_min_x = i_min(dim_min_x, x);
    dim_max_x = i_max(dim_max_x, x);
    dim_min_y = i_min(dim_min_y, y);
    dim_max_y = i_max(dim_max_y, y);
    dim_min_z = i_min(dim_min_z, z);
    dim_max_z = i_max(dim_max_z, z);
  }
}

template <typename T>
inline void i_get_pointclouds_dim(const T *threeds, int n, int start_offset,
                                  int element_size, T &dim_min_x, T &dim_max_x,
                                  T &dim_min_y, T &dim_max_y, T &dim_min_z,
                                  T &dim_max_z) {
  int i;
  T x, y, z;
  dim_min_x = dim_min_y = dim_min_z = Constant<T>::MAX_VAL() / 2;
  dim_max_x = dim_max_y = dim_max_z = -(Constant<T>::MAX_VAL() / 2);
  const T *cptr = threeds + start_offset;
  for (i = 0; i < n; i++) {
    x = cptr[0];
    y = cptr[1];
    z = cptr[2];
    dim_min_x = i_min(dim_min_x, x);
    dim_max_x = i_max(dim_max_x, x);
    dim_min_y = i_min(dim_min_y, y);
    dim_max_y = i_max(dim_max_y, y);
    dim_min_z = i_min(dim_min_z, z);
    dim_max_z = i_max(dim_max_z, z);
    cptr += element_size;
  }
}

template <typename T>
inline void i_get_pointclouds_dim_xy(const T *threeds, int n, int start_offset,
                                     int element_size, T &dim_min_x,
                                     T &dim_max_x, T &dim_min_y, T &dim_max_y) {
  int i;
  T x, y;
  dim_min_x = dim_min_y = Constant<T>::MAX_VAL() / 2;
  dim_max_x = dim_max_y = -(Constant<T>::MAX_VAL() / 2);
  const T *cptr = threeds + start_offset;
  for (i = 0; i < n; i++) {
    x = cptr[0];
    y = cptr[1];
    dim_min_x = i_min(dim_min_x, x);
    dim_max_x = i_max(dim_max_x, x);
    dim_min_y = i_min(dim_min_y, y);
    dim_max_y = i_max(dim_max_y, y);
    cptr += element_size;
  }
}

template <typename T>
inline void i_get_pointclouds_dim_w_bound(
    const T *threeds, int n, T &dim_min_x, T &dim_max_x, T &dim_min_y,
    T &dim_max_y, T &dim_min_z, T &dim_max_z, T bound_min_x, T bound_max_x,
    T bound_min_y, T bound_max_y, T bound_min_z, T bound_max_z) {
  int i, n3 = n * 3;
  T x, y, z;
  dim_min_x = dim_min_y = dim_min_z = Constant<T>::MAX_VAL() / 2;
  dim_max_x = dim_max_y = dim_max_z = -(Constant<T>::MAX_VAL() / 2);
  for (i = 0; i < n3;) {
    x = threeds[i++];
    y = threeds[i++];
    z = threeds[i++];

    if (x < bound_min_x || x > bound_max_x || y < bound_min_y ||
        y > bound_max_y || z < bound_min_z || z > bound_max_z) {
      continue;
    }

    dim_min_x = i_min(dim_min_x, x);
    dim_max_x = i_max(dim_max_x, x);
    dim_min_y = i_min(dim_min_y, y);
    dim_max_y = i_max(dim_max_y, y);
    dim_min_z = i_min(dim_min_z, z);
    dim_max_z = i_max(dim_max_z, z);
  }
}

template <typename T>
inline void i_get_pointclouds_dim_w_bound(
    const T *threeds, int n, int start_offset, int element_size, T &dim_min_x,
    T &dim_max_x, T &dim_min_y, T &dim_max_y, T &dim_min_z, T &dim_max_z,
    T bound_min_x, T bound_max_x, T bound_min_y, T bound_max_y, T bound_min_z,
    T bound_max_z) {
  int i;
  T x, y, z;
  dim_min_x = dim_min_y = dim_min_z = Constant<T>::MAX_VAL() / 2;
  dim_max_x = dim_max_y = dim_max_z = -(Constant<T>::MAX_VAL() / 2);
  const T *cptr = threeds + start_offset;
  for (i = 0; i < n; i++) {
    x = cptr[0];
    y = cptr[1];
    z = cptr[2];
    cptr += element_size;

    if (x < bound_min_x || x > bound_max_x || y < bound_min_y ||
        y > bound_max_y || z < bound_min_z || z > bound_max_z) {
      continue;
    } else {
      dim_min_x = i_min(dim_min_x, x);
      dim_max_x = i_max(dim_max_x, x);
      dim_min_y = i_min(dim_min_y, y);
      dim_max_y = i_max(dim_max_y, y);
      dim_min_z = i_min(dim_min_z, z);
      dim_max_z = i_max(dim_max_z, z);
    }
  }
}

template <typename T>
inline void i_normalize_3d_points_xy_dim(T *threeds, int n, int start_offset,
                                         int element_size,
                                         T max_range_span_radius) {
  int i;
  T dim_min_x, dim_max_x, dim_min_y, dim_max_y;
  T span_x, span_y, span_max, sf;
  T *ptr = threeds + start_offset;

  i_get_pointclouds_dim_xy((const T *)threeds, n, start_offset, element_size,
                           dim_min_x, dim_max_x, dim_min_y, dim_max_y);

  span_x = (dim_max_x - dim_min_x) / 2;
  span_y = (dim_max_y - dim_min_y) / 2;
  span_max = i_max(span_x, span_y);

  sf = i_div(max_range_span_radius, span_max);

  /*rescale object down to [-max_range_span_radius, max_range_span_radius]:*/
  span_x = -(span_x + dim_min_x);
  span_y = -(span_y + dim_min_y);

  if (span_max > max_range_span_radius) {
    for (i = 0; i < n; ++i) {
      ptr[0] = (ptr[0] + span_x) * sf;
      ptr[1] = (ptr[1] + span_y) * sf;
      ptr += element_size;
    }
  } else {
    for (i = 0; i < n; ++i) {
      ptr[0] = (ptr[0] + span_x);
      ptr[1] = (ptr[1] + span_y);
      ptr += element_size;
    }
  }
}

template <typename T>
inline void i_normalize_3d_points_xy_dim(const T *threeds, T *vec3_xyz, int n,
                                         int start_offset, int element_size,
                                         T max_range_span_radius) {
  int i;
  T dim_min_x, dim_max_x, dim_min_y, dim_max_y;
  T span_x, span_y, span_max, sf;
  const T *cptr = threeds + start_offset;
  T *ptr = vec3_xyz;

  i_get_pointclouds_dim_xy(threeds, n, start_offset, element_size, dim_min_x,
                           dim_max_x, dim_min_y, dim_max_y);

  span_x = (dim_max_x - dim_min_x) / 2;
  span_y = (dim_max_y - dim_min_y) / 2;
  span_max = i_max(span_x, span_y);

  sf = i_div(max_range_span_radius, span_max);

  /*rescale object down to [-max_range_span_radius, max_range_span_radius]:*/
  span_x = -(span_x + dim_min_x);
  span_y = -(span_y + dim_min_y);

  if (span_max > max_range_span_radius) {
    for (i = 0; i < n; ++i) {
      ptr[0] = (cptr[0] + span_x) * sf;
      ptr[1] = (cptr[1] + span_y) * sf;
      ptr[2] = cptr[2];
      cptr += element_size;
      ptr += 3;
    }
  } else {
    for (i = 0; i < n; ++i) {
      ptr[0] = (cptr[0] + span_x);
      ptr[1] = (cptr[1] + span_y);
      ptr[2] = cptr[2];
      cptr += element_size;
      ptr += 3;
    }
  }
}

template <typename T>
inline void i_normalize_3d_points(T **threeds, T H[4],
                                  T scene_range_span_radius, int n) {
  int i;
  T min_x, min_y, min_z;
  T max_x, max_y, max_z;
  T span_x, span_y, span_z, sf;
  min_x = min_y = min_z = Constant<T>::MAX_VAL();
  max_x = max_y = max_z = -(Constant<T>::MAX_VAL() / 2);
  for (i = 0; i < n; ++i) {
    min_x = i_min(min_x, threeds[i][0]);
    max_x = i_max(max_x, threeds[i][0]);
    min_y = i_min(min_y, threeds[i][1]);
    max_y = i_max(max_y, threeds[i][1]);
    min_z = i_min(min_z, threeds[i][2]);
    max_z = i_max(max_z, threeds[i][2]);
  }

  span_x = (max_x - min_x) / 2;
  span_y = (max_y - min_y) / 2;
  span_z = (max_z - min_z) / 2;

  /*rescale object down to [-scene_range_span_radius,
   * scene_range_span_radius]:*/
  sf = i_min(i_min(i_div((T)scene_range_span_radius, span_x),
                   i_div((T)scene_range_span_radius, span_y)),
             i_div((T)scene_range_span_radius, span_z));
  span_x += min_x;
  span_y += min_y;
  span_z += min_z;
  for (i = 0; i < n; ++i) {
    threeds[i][0] = (threeds[i][0] - span_x) * sf;
    threeds[i][1] = (threeds[i][1] - span_y) * sf;
    threeds[i][2] = (threeds[i][2] - span_z) * sf;
  }
  H[0] = span_x;
  H[1] = span_y;
  H[2] = span_z;
  H[3] = sf;
}
} /*namespace idl*/

#endif