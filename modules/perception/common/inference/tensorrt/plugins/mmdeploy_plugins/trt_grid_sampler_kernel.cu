/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <cuda_fp16.h>
#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/common_cuda_helper.hpp"
#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_grid_sampler_kernel.hpp"
#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_plugin_helper.hpp"

namespace apollo {
namespace perception {
namespace inference {

// Unnormalizes a coordinate from the -1 to +1 scale to its pixel index value,
// where we view each pixel as an area between (idx - 0.5) and (idx + 0.5).
// if align_corners: -1 and +1 get sent to the centers of the corner pixels
//     -1 --> 0
//     +1 --> (size - 1)
//     scale_factor = (size - 1) / 2
// if not align_corners: -1 and +1 get sent to the image edges
//     -1 --> -0.5
//     +1 --> (size - 1) + 0.5 == size - 0.5
//     scale_factor = size / 2
template <typename scalar_t>
static __forceinline__ __device__ scalar_t grid_sampler_unnormalize(scalar_t coord, int size,
                                                                    bool align_corners) {
  if (align_corners) {
    // unnormalize coord from [-1, 1] to [0, size - 1]
    return ((coord + 1.f) / 2) * (size - 1);
  } else {
    // unnormalize coord from [-1, 1] to [-0.5, size - 0.5]
    return ((coord + 1.f) * size - 1) / 2;
  }
}

// Clips coordinates to between 0 and clip_limit - 1
template <typename scalar_t>
static __forceinline__ __device__ scalar_t clip_coordinates(scalar_t in, int clip_limit) {
  return ::min(static_cast<scalar_t>(clip_limit - 1), ::max(in, static_cast<scalar_t>(0)));
}

// Reflects coordinates until they fall between low and high (inclusive).
// The bounds are passed as twice their value so that half-integer values
// can be represented as ints.
template <typename scalar_t>
static __forceinline__ __device__ scalar_t reflect_coordinates(scalar_t in, int twice_low,
                                                               int twice_high) {
  if (twice_low == twice_high) {
    return static_cast<scalar_t>(0);
  }
  scalar_t min = static_cast<scalar_t>(twice_low) / 2;
  scalar_t span = static_cast<scalar_t>(twice_high - twice_low) / 2;
  in = ::fabs(in - min);
  // `fmod` returns same sign as `in`, which is positive after the `fabs` above.
  scalar_t extra = ::fmod(in, span);
  int flips = static_cast<int>(::floor(in / span));
  if (flips % 2 == 0) {
    return extra + min;
  } else {
    return span - extra + min;
  }
}

template <typename scalar_t>
static __forceinline__ __device__ scalar_t safe_downgrade_to_int_range(scalar_t x) {
  // -100.0 does not have special meaning. This is just to make sure
  // it's not within_bounds_2d or within_bounds_3d, and does not cause
  // undefined behavior. See #35506.
  if (x > INT_MAX - 1 || x < INT_MIN || !::isfinite(static_cast<double>(x)))
    return static_cast<scalar_t>(-100.0);
  return x;
}

// Computes the pixel source index value for a grid coordinate
template <typename scalar_t>
static __forceinline__ __device__ scalar_t grid_sampler_compute_source_index(
    scalar_t coord, int size, GridSamplerPadding padding_mode, bool align_corners) {
  coord = grid_sampler_unnormalize(coord, size, align_corners);
  if (padding_mode == GridSamplerPadding::Border) {
    // clip coordinates to image borders
    coord = clip_coordinates(coord, size);
  } else if (padding_mode == GridSamplerPadding::Reflection) {
    // reflect coordinates by image borders
    if (align_corners) {
      coord = reflect_coordinates(coord, 0, 2 * (size - 1));
    } else {
      coord = reflect_coordinates(coord, -1, 2 * size - 1);
    }
    // clip coordinates to image borders
    coord = clip_coordinates(coord, size);
  }

  coord = safe_downgrade_to_int_range(coord);
  return coord;
}

static __forceinline__ __device__ bool within_bounds_2d(int h, int w, int H, int W) {
  return h >= 0 && h < H && w >= 0 && w < W;
}

static __forceinline__ __device__ bool within_bounds_3d(int d, int h, int w, int D, int H, int W) {
  return d >= 0 && d < D && h >= 0 && h < H && w >= 0 && w < W;
}

template <typename scalar_t>
__global__ void grid_sampler_2d_kernel(const int nthreads, const scalar_t *input,
                                       const scalar_t *grid, scalar_t *output,
                                       TensorDesc input_desc, TensorDesc grid_desc,
                                       TensorDesc output_desc,
                                       const GridSamplerInterpolation interpolation_mode,
                                       const GridSamplerPadding padding_mode, bool align_corners) {
  int C = input_desc.shape[1];
  int inp_H = input_desc.shape[2];
  int inp_W = input_desc.shape[3];
  int out_H = grid_desc.shape[1];
  int out_W = grid_desc.shape[2];
  int inp_sN = input_desc.stride[0];
  int inp_sC = input_desc.stride[1];
  int inp_sH = input_desc.stride[2];
  int inp_sW = input_desc.stride[3];
  int grid_sN = grid_desc.stride[0];
  int grid_sH = grid_desc.stride[1];
  int grid_sW = grid_desc.stride[2];
  int grid_sCoor = grid_desc.stride[3];
  int out_sN = output_desc.stride[0];
  int out_sC = output_desc.stride[1];
  int out_sH = output_desc.stride[2];
  int out_sW = output_desc.stride[3];

  CUDA_1D_KERNEL_LOOP(index, nthreads) {
    const int w = index % out_W;
    const int h = (index / out_W) % out_H;
    const int n = index / (out_H * out_W);
    const int grid_offset = n * grid_sN + h * grid_sH + w * grid_sW;

    // get the corresponding input x, y coordinates from grid
    scalar_t ix = grid[grid_offset];
    scalar_t iy = grid[grid_offset + grid_sCoor];

    ix = grid_sampler_compute_source_index(ix, inp_W, padding_mode, align_corners);
    iy = grid_sampler_compute_source_index(iy, inp_H, padding_mode, align_corners);

    if (interpolation_mode == GridSamplerInterpolation::Bilinear) {
      // get NE, NW, SE, SW pixel values from (x, y)
      int ix_nw = static_cast<int>(::floor(ix));
      int iy_nw = static_cast<int>(::floor(iy));
      int ix_ne = ix_nw + 1;
      int iy_ne = iy_nw;
      int ix_sw = ix_nw;
      int iy_sw = iy_nw + 1;
      int ix_se = ix_nw + 1;
      int iy_se = iy_nw + 1;

      // get surfaces to each neighbor:
      scalar_t nw = (ix_se - ix) * (iy_se - iy);
      scalar_t ne = (ix - ix_sw) * (iy_sw - iy);
      scalar_t sw = (ix_ne - ix) * (iy - iy_ne);
      scalar_t se = (ix - ix_nw) * (iy - iy_nw);

      // calculate bilinear weighted pixel value and set output pixel
      auto inp_ptr_NC = input + n * inp_sN;
      auto out_ptr_NCHW = output + n * out_sN + h * out_sH + w * out_sW;
      for (int c = 0; c < C; ++c, inp_ptr_NC += inp_sC, out_ptr_NCHW += out_sC) {
        *out_ptr_NCHW = static_cast<scalar_t>(0);
        if (within_bounds_2d(iy_nw, ix_nw, inp_H, inp_W)) {
          *out_ptr_NCHW += inp_ptr_NC[iy_nw * inp_sH + ix_nw * inp_sW] * nw;
        }
        if (within_bounds_2d(iy_ne, ix_ne, inp_H, inp_W)) {
          *out_ptr_NCHW += inp_ptr_NC[iy_ne * inp_sH + ix_ne * inp_sW] * ne;
        }
        if (within_bounds_2d(iy_sw, ix_sw, inp_H, inp_W)) {
          *out_ptr_NCHW += inp_ptr_NC[iy_sw * inp_sH + ix_sw * inp_sW] * sw;
        }
        if (within_bounds_2d(iy_se, ix_se, inp_H, inp_W)) {
          *out_ptr_NCHW += inp_ptr_NC[iy_se * inp_sH + ix_se * inp_sW] * se;
        }
      }
    } else if (interpolation_mode == GridSamplerInterpolation::Nearest) {
      int ix_nearest = static_cast<int>(::round(ix));
      int iy_nearest = static_cast<int>(::round(iy));

      // assign nearest neighbor pixel value to output pixel
      auto inp_ptr_NC = input + n * inp_sN;
      auto out_ptr_NCHW = output + n * out_sN + h * out_sH + w * out_sW;
      for (int c = 0; c < C; ++c, inp_ptr_NC += inp_sC, out_ptr_NCHW += out_sC) {
        if (within_bounds_2d(iy_nearest, ix_nearest, inp_H, inp_W)) {
          *out_ptr_NCHW = inp_ptr_NC[iy_nearest * inp_sH + ix_nearest * inp_sW];
        } else {
          *out_ptr_NCHW = static_cast<scalar_t>(0);
        }
      }
    }
  }
}

template <typename scalar_t>
__global__ void grid_sampler_3d_kernel(const int nthreads, const scalar_t *input,
                                       const scalar_t *grid, scalar_t *output,
                                       TensorDesc input_desc, TensorDesc grid_desc,
                                       TensorDesc output_desc,
                                       const GridSamplerInterpolation interpolation_mode,
                                       const GridSamplerPadding padding_mode, bool align_corners) {
  int C = input_desc.shape[1];
  int inp_D = input_desc.shape[2];
  int inp_H = input_desc.shape[3];
  int inp_W = input_desc.shape[4];
  int out_D = grid_desc.shape[1];
  int out_H = grid_desc.shape[2];
  int out_W = grid_desc.shape[3];
  int inp_sN = input_desc.stride[0];
  int inp_sC = input_desc.stride[1];
  int inp_sD = input_desc.stride[2];
  int inp_sH = input_desc.stride[3];
  int inp_sW = input_desc.stride[4];
  int grid_sN = grid_desc.stride[0];
  int grid_sD = grid_desc.stride[1];
  int grid_sH = grid_desc.stride[2];
  int grid_sW = grid_desc.stride[3];
  int grid_sCoor = grid_desc.stride[4];
  int out_sN = output_desc.stride[0];
  int out_sC = output_desc.stride[1];
  int out_sD = output_desc.stride[2];
  int out_sH = output_desc.stride[3];
  int out_sW = output_desc.stride[4];

  CUDA_1D_KERNEL_LOOP(index, nthreads) {
    const int w = index % out_W;
    const int h = (index / out_W) % out_H;
    const int d = (index / (out_H * out_W)) % out_D;
    const int n = index / (out_D * out_H * out_W);
    const int grid_offset = n * grid_sN + d * grid_sD + h * grid_sH + w * grid_sW;

    // get the corresponding input x, y, z coordinates from grid
    scalar_t ix = grid[grid_offset];
    scalar_t iy = grid[grid_offset + grid_sCoor];
    scalar_t iz = grid[grid_offset + 2 * grid_sCoor];

    ix = grid_sampler_compute_source_index(ix, inp_W, padding_mode, align_corners);
    iy = grid_sampler_compute_source_index(iy, inp_H, padding_mode, align_corners);
    iz = grid_sampler_compute_source_index(iz, inp_D, padding_mode, align_corners);

    if (interpolation_mode == GridSamplerInterpolation::Bilinear) {
      // get corner pixel values from (x, y, z)
      // for 4d, we used north-east-south-west
      // for 5d, we add top-bottom
      int ix_tnw = static_cast<int>(::floor(ix));
      int iy_tnw = static_cast<int>(::floor(iy));
      int iz_tnw = static_cast<int>(::floor(iz));

      int ix_tne = ix_tnw + 1;
      int iy_tne = iy_tnw;
      int iz_tne = iz_tnw;

      int ix_tsw = ix_tnw;
      int iy_tsw = iy_tnw + 1;
      int iz_tsw = iz_tnw;

      int ix_tse = ix_tnw + 1;
      int iy_tse = iy_tnw + 1;
      int iz_tse = iz_tnw;

      int ix_bnw = ix_tnw;
      int iy_bnw = iy_tnw;
      int iz_bnw = iz_tnw + 1;

      int ix_bne = ix_tnw + 1;
      int iy_bne = iy_tnw;
      int iz_bne = iz_tnw + 1;

      int ix_bsw = ix_tnw;
      int iy_bsw = iy_tnw + 1;
      int iz_bsw = iz_tnw + 1;

      int ix_bse = ix_tnw + 1;
      int iy_bse = iy_tnw + 1;
      int iz_bse = iz_tnw + 1;

      // get surfaces to each neighbor:
      scalar_t tnw = (ix_bse - ix) * (iy_bse - iy) * (iz_bse - iz);
      scalar_t tne = (ix - ix_bsw) * (iy_bsw - iy) * (iz_bsw - iz);
      scalar_t tsw = (ix_bne - ix) * (iy - iy_bne) * (iz_bne - iz);
      scalar_t tse = (ix - ix_bnw) * (iy - iy_bnw) * (iz_bnw - iz);
      scalar_t bnw = (ix_tse - ix) * (iy_tse - iy) * (iz - iz_tse);
      scalar_t bne = (ix - ix_tsw) * (iy_tsw - iy) * (iz - iz_tsw);
      scalar_t bsw = (ix_tne - ix) * (iy - iy_tne) * (iz - iz_tne);
      scalar_t bse = (ix - ix_tnw) * (iy - iy_tnw) * (iz - iz_tnw);

      auto inp_ptr_NC = input + n * inp_sN;
      auto out_ptr_NCDHW = output + n * out_sN + d * out_sD + h * out_sH + w * out_sW;
      for (int c = 0; c < C; ++c, inp_ptr_NC += inp_sC, out_ptr_NCDHW += out_sC) {
        //   (c, iz_tnw, iy_tnw, ix_tnw) * tnw + (c, iz_tne, iy_tne, ix_tne) *
        //   tne
        // + (c, iz_tsw, iy_tsw, ix_tsw) * tsw + (c, iz_tse, iy_tse, ix_tse) *
        // tse
        // + (c, iz_bnw, iy_bnw, ix_bnw) * bnw + (c, iz_bne, iy_bne, ix_bne) *
        // bne
        // + (c, iz_bsw, iy_bsw, ix_bsw) * bsw + (c, iz_bse, iy_bse, ix_bse) *
        // bse
        *out_ptr_NCDHW = static_cast<scalar_t>(0);
        if (within_bounds_3d(iz_tnw, iy_tnw, ix_tnw, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_tnw * inp_sD + iy_tnw * inp_sH + ix_tnw * inp_sW] * tnw;
        }
        if (within_bounds_3d(iz_tne, iy_tne, ix_tne, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_tne * inp_sD + iy_tne * inp_sH + ix_tne * inp_sW] * tne;
        }
        if (within_bounds_3d(iz_tsw, iy_tsw, ix_tsw, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_tsw * inp_sD + iy_tsw * inp_sH + ix_tsw * inp_sW] * tsw;
        }
        if (within_bounds_3d(iz_tse, iy_tse, ix_tse, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_tse * inp_sD + iy_tse * inp_sH + ix_tse * inp_sW] * tse;
        }
        if (within_bounds_3d(iz_bnw, iy_bnw, ix_bnw, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_bnw * inp_sD + iy_bnw * inp_sH + ix_bnw * inp_sW] * bnw;
        }
        if (within_bounds_3d(iz_bne, iy_bne, ix_bne, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_bne * inp_sD + iy_bne * inp_sH + ix_bne * inp_sW] * bne;
        }
        if (within_bounds_3d(iz_bsw, iy_bsw, ix_bsw, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_bsw * inp_sD + iy_bsw * inp_sH + ix_bsw * inp_sW] * bsw;
        }
        if (within_bounds_3d(iz_bse, iy_bse, ix_bse, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW += inp_ptr_NC[iz_bse * inp_sD + iy_bse * inp_sH + ix_bse * inp_sW] * bse;
        }
      }
    } else if (interpolation_mode == GridSamplerInterpolation::Nearest) {
      int ix_nearest = static_cast<int>(::round(ix));
      int iy_nearest = static_cast<int>(::round(iy));
      int iz_nearest = static_cast<int>(::round(iz));

      // assign nearest neighbor pixel value to output pixel
      auto inp_ptr_NC = input + n * inp_sN;
      auto out_ptr_NCDHW = output + n * out_sN + d * out_sD + h * out_sH + w * out_sW;
      for (int c = 0; c < C; ++c, inp_ptr_NC += inp_sC, out_ptr_NCDHW += out_sC) {
        if (within_bounds_3d(iz_nearest, iy_nearest, ix_nearest, inp_D, inp_H, inp_W)) {
          *out_ptr_NCDHW =
              inp_ptr_NC[iz_nearest * inp_sD + iy_nearest * inp_sH + ix_nearest * inp_sW];
        } else {
          *out_ptr_NCDHW = static_cast<scalar_t>(0);
        }
      }
    }
  }
}

void create_desc(const int *dims, int nb_dims, TensorDesc &desc) {
  memcpy(&desc.shape[0], dims, sizeof(int) * nb_dims);
  desc.stride[nb_dims - 1] = 1;
  for (int i = nb_dims - 2; i >= 0; --i) {
    desc.stride[i] = desc.stride[i + 1] * desc.shape[i + 1];
  }
}

template <typename T>
void grid_sample(T *output, const T *input, const T *grid, int *output_dims, int *input_dims,
                 int *grid_dims, int nb_dims, GridSamplerInterpolation interp,
                 GridSamplerPadding padding, bool align_corners, cudaStream_t stream) {
  TensorDesc input_desc;
  create_desc(input_dims, nb_dims, input_desc);

  TensorDesc output_desc;
  create_desc(output_dims, nb_dims, output_desc);

  TensorDesc grid_desc;
  create_desc(grid_dims, nb_dims, grid_desc);

  int count = 1;
  for (int i = 0; i < nb_dims; ++i) {
    if (i == 1) {
      continue;
    }
    count *= output_desc.shape[i];
  }

  if (nb_dims == 4) {
    grid_sampler_2d_kernel<T><<<GET_BLOCKS(count), THREADS_PER_BLOCK, 0, stream>>>(
        count, input, grid, output, input_desc, grid_desc, output_desc, interp, padding,
        align_corners);
  } else if (nb_dims == 5) {
    grid_sampler_3d_kernel<T><<<GET_BLOCKS(count), THREADS_PER_BLOCK, 0, stream>>>(
        count, input, grid, output, input_desc, grid_desc, output_desc, interp, padding,
        align_corners);
  } else {
    printf("input and grid dims should be 4 or 5\n");
  }
}

template void grid_sample<float>(float *output, const float *input, const float *grid,
                                 int *output_dims, int *input_dims, int *grid_dims, int nb_dims,
                                 GridSamplerInterpolation interp, GridSamplerPadding padding,
                                 bool align_corners, cudaStream_t stream);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
