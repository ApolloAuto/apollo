/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

namespace apollo {
namespace perception {
namespace camera {

const uint32_t THREADS_PER_BLOCK_X = 32;
const uint32_t THREADS_PER_BLOCK_Y = 32;

template <typename T>
struct image2D {
  unsigned char *data;
  size_t width_step;
  __device__ image2D(const T *data_pointer, size_t width_step)
      : width_step(width_step) {
    data = reinterpret_cast<unsigned char *>(const_cast<T *>(data_pointer));
  }
  inline __device__ T &operator()(const size_t i, const size_t j) {
    return *(reinterpret_cast<T *>(data + width_step * j + i * sizeof(T)));
  }
  inline __device__ const T &operator()(const size_t i, const size_t j) const {
    return *(reinterpret_cast<T *>(data + width_step * j + i * sizeof(T)));
  }
};

bool rppInitDescriptor(RpptDescPtr &descPtr, int width, int height,
                       int channels, int width_step) {
  descPtr->dataType = RpptDataType::U8;
  descPtr->numDims = 4;
  descPtr->offsetInBytes = 0;
  descPtr->n = 1;
  descPtr->h = height;
  descPtr->w = width;
  descPtr->c = channels;
  switch (channels) {
    case 1:
      descPtr->layout = RpptLayout::NCHW;
      descPtr->strides.wStride = 1;
      descPtr->strides.hStride = width_step;
      descPtr->strides.cStride = descPtr->strides.hStride * descPtr->h;
      descPtr->strides.nStride =
          descPtr->strides.hStride * descPtr->h * descPtr->c;
      break;
    case 3:
      descPtr->layout = RpptLayout::NHWC;
      descPtr->strides.cStride = 1;
      descPtr->strides.wStride = descPtr->c;
      descPtr->strides.hStride = width_step;
      descPtr->strides.nStride = descPtr->strides.hStride * descPtr->h;
      break;
    default:
      AERROR << "Invalid number of channels: " << channels
             << "; only 1 and 3 are supported.";
      return false;
  }
  return true;
}

bool rppImageToBlob(const base::Image8U &image, base::Blob<uint8_t> *blob) {
  RpptDesc srcDesc, dstDesc;
  RpptDescPtr srcDescPtr = &srcDesc, dstDescPtr = &dstDesc;

  blob->Reshape({1, image.rows(), image.cols(), image.channels()});

  if (!rppInitDescriptor(srcDescPtr, image.cols(), image.rows(),
                         image.channels(), image.width_step()))
    return false;
  if (!rppInitDescriptor(dstDescPtr, image.cols(), image.rows(), image.channels(),
                         blob->count(2) * static_cast<int>(sizeof(uint8_t))))
    return false;

  rppHandle_t handle;
  rppCreateWithBatchSize(&handle, 1);
  RppStatus status =
    rppt_copy_gpu((const_cast<base::Image8U &>(image)).mutable_gpu_data(),
                  srcDescPtr, blob->mutable_gpu_data(), dstDescPtr, handle);
  if (status != RPP_SUCCESS)
    return false;
  return true;
}

bool rppImageToGray(const base::Image8UPtr &src, const base::Image8UPtr &dst,
                    const int src_width, const int src_height,
                    const float coeffs[3]) {
  RppStatus status = RPP_SUCCESS;
  RpptDesc srcDesc, dstDesc;
  RpptDescPtr srcDescPtr = &srcDesc, dstDescPtr = &dstDesc;

  if (!rppInitDescriptor(srcDescPtr, src_width, src_height, 3,
                         src->width_step()))
    return false;
  if (!rppInitDescriptor(dstDescPtr, src_width, src_height, 1,
                         dst->width_step()))
    return false;

  rppHandle_t handle;
  rppCreateWithBatchSize(&handle, 1);
  assert((coeffs[1] == 0.587f) &&
         ((coeffs[0] == 0.114f && coeffs[2] == 0.299f) ||
          (coeffs[0] == 0.299f && coeffs[2] == 0.114f)) &&
         "coefficients in rppt_color_to_greyscale_gpu are hardcoded");
  // BGR: float coeffs[] = {0.114f, 0.587f, 0.299f};
  if (coeffs[0] == 0.114f && coeffs[1] == 0.587f && coeffs[2] == 0.299f) {
    status = rppt_color_to_greyscale_gpu(src->mutable_gpu_data(), srcDescPtr,
                                         dst->mutable_gpu_data(), dstDescPtr,
                                         RpptSubpixelLayout::BGRtype, handle);
  }
  // RGB: float coeffs[] = {0.299f, 0.587f, 0.114f};
  if (coeffs[0] == 0.299f && coeffs[1] == 0.587f && coeffs[2] == 0.114f) {
    status = rppt_color_to_greyscale_gpu(src->mutable_gpu_data(), srcDescPtr,
                                         dst->mutable_gpu_data(), dstDescPtr,
                                         RpptSubpixelLayout::RGBtype, handle);
  }
  if (status != RPP_SUCCESS)
    return false;
  return true;
}

bool rppSwapImageChannels(const base::Image8UPtr &src,
                          const base::Image8UPtr &dst,
                          const int src_width, const int src_height,
                          const int order[3]) {
  RpptDesc srcDesc, dstDesc;
  RpptDescPtr srcDescPtr = &srcDesc, dstDescPtr = &dstDesc;

  if (!rppInitDescriptor(srcDescPtr, src_width, src_height, 3,
                         src->width_step()))
    return false;
  if (!rppInitDescriptor(dstDescPtr, src_width, src_height, 3,
                         dst->width_step()))
    return false;

  rppHandle_t handle;
  rppCreateWithBatchSize(&handle, 1);
  assert(order[0] == 2 && order[1] == 1 && order[2] == 0 &&
         "The order in rppt_swap_channels is hardcoded");
  RppStatus status =
      rppt_swap_channels_gpu(src->mutable_gpu_data(), srcDescPtr,
                             dst->mutable_gpu_data(), dstDescPtr, handle);
  if (status != RPP_SUCCESS)
    return false;
  return true;
}

__global__ void duplicate_kernel(const unsigned char *src,
                                 size_t src_width_step, uchar3 *dst,
                                 size_t dst_width_step, int width, int height) {
  const size_t i = hipBlockDim_x * hipBlockIdx_x + hipThreadIdx_x;
  const size_t j = hipBlockDim_y * hipBlockIdx_y + hipThreadIdx_y;
  image2D<unsigned char> src_img{src, src_width_step};
  image2D<uchar3> dst_img{dst, dst_width_step};

  if (i < width && j < height) {
    unsigned char value = src_img(i, j);
    dst_img(i, j).x = value;
    dst_img(i, j).y = value;
    dst_img(i, j).z = value;
  }
}

bool rppDupImageChannels(const base::Image8UPtr &src,
                         const base::Image8UPtr &dst,
                         const int src_width, const int src_height) {
  dim3 threadsPerBlock(THREADS_PER_BLOCK_X, THREADS_PER_BLOCK_Y);
  dim3 blocks((src_width + threadsPerBlock.x - 1) / threadsPerBlock.x,
              (src_height + threadsPerBlock.y - 1) / threadsPerBlock.y);

  hipLaunchKernelGGL(duplicate_kernel, blocks, threadsPerBlock, 0, 0,
                     src->gpu_data(), src->width_step(),
                     reinterpret_cast<uchar3 *>(dst->mutable_gpu_data()),
                     dst->width_step(), src_width, src_height);

  if (hipSuccess != hipGetLastError())
    return false;
  return true;
}

__global__ void remap_pln1_kernel(const unsigned char *src,
                                  size_t src_width_step, unsigned char *dst,
                                  size_t dst_width_step, const float *mapx,
                                  const float *mapy, int width, int height) {
  const size_t i = hipBlockDim_x * hipBlockIdx_x + hipThreadIdx_x;
  const size_t j = hipBlockDim_y * hipBlockIdx_y + hipThreadIdx_y;
  image2D<unsigned char> src_img{src, src_width_step};
  image2D<unsigned char> dst_img{dst, dst_width_step};
  if (i < width && j < height) {
    float x_coor = mapx[j * width + i];
    float y_coor = mapy[j * width + i];

    int X = trunc(x_coor);
    int Y = trunc(y_coor);
    float x_frac = x_coor - X;
    float y_frac = y_coor - Y;

    if (0 <= X && X < width && 0 <= Y && Y < height) {
      // uchar p[2][2];
      int X1 = (X < width - 1) ? X + 1 : X;
      int Y1 = (Y < height - 1) ? Y + 1 : Y;

      unsigned char pixel00 = src_img(X, Y);
      unsigned char pixel01 = src_img(X1, Y);
      unsigned char pixel10 = src_img(X, Y1);
      unsigned char pixel11 = src_img(X1, Y1);
      // bilinear interpolation
      unsigned char interpolated =
          (pixel00 * (1 - x_frac) + pixel01 * x_frac) * (1 - y_frac) +
          (pixel10 * (1 - x_frac) + pixel11 * x_frac) * y_frac;
      dst_img(i, j) = interpolated;
    }
  }
}

__global__ void remap_pkd3_kernel(const uchar3 *src, size_t src_width_step,
                                  uchar3 *dst, size_t dst_width_step,
                                  const float *mapx, const float *mapy,
                                  int width, int height) {
  const size_t i = hipBlockDim_x * hipBlockIdx_x + hipThreadIdx_x;
  const size_t j = hipBlockDim_y * hipBlockIdx_y + hipThreadIdx_y;

  image2D<uchar3> src_img{src, src_width_step};
  image2D<uchar3> dst_img{dst, dst_width_step};

  if (i < width && j < height) {
    float x_coor = mapx[j * width + i];
    float y_coor = mapy[j * width + i];

    int X = trunc(x_coor);
    int Y = trunc(y_coor);
    float x_frac = x_coor - X;
    float y_frac = y_coor - Y;

    if (0 <= X && X < width && 0 <= Y && Y < height) {
      // uchar3 p[2][2];
      int X1 = (X < width - 1) ? X + 1 : X;
      int Y1 = (Y < height - 1) ? Y + 1 : Y;

      uchar3 pixel00 = src_img(X, Y);
      uchar3 pixel01 = src_img(X1, Y);
      uchar3 pixel10 = src_img(X, Y1);
      uchar3 pixel11 = src_img(X1, Y1);
      // bilinear interpolation
      uchar3 interpolated;
      interpolated.x =
          (pixel00.x * (1 - x_frac) + pixel01.x * x_frac) * (1 - y_frac) +
          (pixel10.x * (1 - x_frac) + pixel11.x * x_frac) * y_frac;
      interpolated.y =
          (pixel00.y * (1 - x_frac) + pixel01.y * x_frac) * (1 - y_frac) +
          (pixel10.y * (1 - x_frac) + pixel11.y * x_frac) * y_frac;
      interpolated.z =
          (pixel00.z * (1 - x_frac) + pixel01.z * x_frac) * (1 - y_frac) +
          (pixel10.z * (1 - x_frac) + pixel11.z * x_frac) * y_frac;

      dst_img(i, j) = interpolated;
    }
  }
}

bool rppImageRemap(const base::Image8U &src_img, base::Image8U *dst_img,
                   const int src_width, const int src_height,
                   const base::Blob<float> &map_x,
                   const base::Blob<float> &map_y) {
  dim3 threadsPerBlock(THREADS_PER_BLOCK_X, THREADS_PER_BLOCK_Y);
  dim3 blocks((src_width + threadsPerBlock.x - 1) / threadsPerBlock.x,
              (src_height + threadsPerBlock.y - 1) / threadsPerBlock.y);

  switch (src_img.channels()) {
    case 1:
      hipLaunchKernelGGL(remap_pln1_kernel, blocks, threadsPerBlock, 0, 0,
                         src_img.gpu_data(), src_img.width_step(),
                         dst_img->mutable_gpu_data(), dst_img->width_step(),
                         map_x.gpu_data(), map_y.gpu_data(), src_width,
                         src_height);
      break;
    case 3:
      hipLaunchKernelGGL(
          remap_pkd3_kernel, blocks, threadsPerBlock, 0, 0,
          reinterpret_cast<const uchar3 *>(src_img.gpu_data()),
          src_img.width_step(),
          reinterpret_cast<uchar3 *>(dst_img->mutable_gpu_data()),
          dst_img->width_step(), map_x.gpu_data(), map_y.gpu_data(), src_width,
          src_height);
      break;
    default:
      AERROR << "Invalid number of channels: " << src_img.channels()
             << "; only 1 and 3 are supported.";
      return false;
  }
  if (hipSuccess != hipGetLastError())
    return false;
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
