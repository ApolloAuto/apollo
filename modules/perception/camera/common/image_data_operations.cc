/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/common/image_data_operations.h"

#include "cyber/common/log.h"

#if GPU_PLATFORM == NVIDIA
#include <nppi.h>
#elif GPU_PLATFORM == AMD
#include <rppi.h>
#endif

namespace apollo {
namespace perception {
namespace camera {

#if GPU_PLATFORM == NVIDIA

bool nppImageToBlob(base::Image8U &image, base::Blob<uint8_t> *blob) {
  NppStatus status;
  NppiSize roi;
  roi.height = image.rows();
  roi.width = image.cols();
  blob->Reshape({1, roi.height, roi.width, image.channels()});
  if (image.channels() == 1) {
    status = nppiCopy_8u_C1R(
        image.gpu_data(), image.width_step(), blob->mutable_gpu_data(),
        blob->count(2) * static_cast<int>(sizeof(uint8_t)), roi);
  } else {
    status = nppiCopy_8u_C3R(
        image.gpu_data(), image.width_step(), blob->mutable_gpu_data(),
        blob->count(2) * static_cast<int>(sizeof(uint8_t)), roi);
  }
  if (status != NPP_SUCCESS) {
    return false;
  }
  return true;
}

bool nppImageToGray(base::Image8UPtr &src, base::Image8UPtr &dst,
                    const int src_width, const int src_height,
                    const float coeffs[3]) {
  NppStatus status;
  NppiSize roi;
  roi.height = src_height;
  roi.width = src_width;
  status = nppiColorToGray_8u_C3C1R(src->gpu_data(), src->width_step(),
                                    dst->mutable_gpu_data(), dst->width_step(),
                                    roi, coeffs);
  if (status != NPP_SUCCESS) {
    return false;
  }
  return true;
}

bool nppSwapImageChannels(base::Image8UPtr &src, base::Image8UPtr &dst,
                          const int src_width, const int src_height,
                          const int order[3]) {
  NppStatus status;
  NppiSize roi;
  roi.height = src_height;
  roi.width = src_width;
  status = nppiSwapChannels_8u_C3R(src->gpu_data(), src->width_step(),
                                   dst->mutable_gpu_data(), dst->width_step(),
                                   roi, order);
  if (status != NPP_SUCCESS) {
    return false;
  }
  return true;
}

bool nppDupImageChannels(base::Image8UPtr &src, base::Image8UPtr &dst,
                         const int src_width, const int src_height) {
  NppStatus status;
  NppiSize roi;
  roi.height = src_height;
  roi.width = src_width;
  status = nppiDup_8u_C1C3R(src->gpu_data(), src->width_step(),
                            dst->mutable_gpu_data(), dst->width_step(), roi);
  if (status != NPP_SUCCESS) {
    return false;
  }
  return true;
}

bool nppImageRemap(const base::Image8U &src_img, base::Image8U *dst_img,
                   const int src_width, const int src_height,
                   const base::Blob<float> &map_x,
                   const base::Blob<float> &map_y) {
  NppiInterpolationMode remap_mode = NPPI_INTER_LINEAR;
  NppiSize image_size;
  image_size.width = src_width;
  image_size.height = src_height;
  NppiRect remap_roi = {0, 0, src_width, src_height};

  NppStatus status = NPP_SUCCESS;
  int d_map_step = static_cast<int>(map_x.shape(1) * sizeof(float));
  switch (src_img.channels()) {
    case 1:
      status = nppiRemap_8u_C1R(src_img.gpu_data(), image_size,
                                src_img.width_step(), remap_roi,
                                map_x.gpu_data(), d_map_step, map_y.gpu_data(),
                                d_map_step, dst_img->mutable_gpu_data(),
                                dst_img->width_step(), image_size, remap_mode);
      break;
    case 3:
      status = nppiRemap_8u_C3R(src_img.gpu_data(), image_size,
                                src_img.width_step(), remap_roi,
                                map_x.gpu_data(), d_map_step, map_y.gpu_data(),
                                d_map_step, dst_img->mutable_gpu_data(),
                                dst_img->width_step(), image_size, remap_mode);
      break;
    default:
      AERROR << "Invalid number of channels: " << src_img.channels();
      return false;
  }

  if (status != NPP_SUCCESS) {
    AERROR << "NPP_CHECK_NPP - status = " << status;
    return false;
  }

  return true;
}

#elif GPU_PLATFORM == AMD

void rppInitDescriptor(RpptDescPtr &descPtr, int width, int height,
                       int channels, int width_step) {
  descPtr->dataType = RpptDataType::U8;
  descPtr->numDims = 4;
  descPtr->offsetInBytes = 0;

  descPtr->n = 1;
  descPtr->h = height;
  descPtr->w = width;
  descPtr->c = channels;

  assert(channels == 1 || channels == 3);
  if (channels == 1) {
    descPtr->layout = RpptLayout::NCHW;
    descPtr->strides.wStride = 1;
    descPtr->strides.hStride = width_step;
    descPtr->strides.cStride = descPtr->strides.hStride * descPtr->h;
    descPtr->strides.nStride =
        descPtr->strides.hStride * descPtr->h * descPtr->c;
  }
  if (channels == 3) {
    descPtr->layout = RpptLayout::NHWC;
    descPtr->strides.cStride = 1;
    descPtr->strides.wStride = descPtr->c;
    descPtr->strides.hStride = width_step;
    descPtr->strides.nStride = descPtr->strides.hStride * descPtr->h;
  }
}

bool rppImageToBlob(base::Image8U &image, base::Blob<uint8_t> *blob) {
  RppStatus status;
  RpptDesc srcDesc, dstDesc;
  RpptDescPtr srcDescPtr, dstDescPtr;
  srcDescPtr = &srcDesc;
  dstDescPtr = &dstDesc;

  blob->Reshape({1, image.rows(), image.cols(), image.channels()});

  rppInitDescriptor(srcDescPtr, image.cols(), image.rows(), image.channels(),
                    image.width_step());
  rppInitDescriptor(dstDescPtr, image.cols(), image.rows(), image.channels(),
                    blob->count(2) * static_cast<int>(sizeof(uint8_t)));

  rppHandle_t handle;
  rppCreateWithBatchSize(&handle, 1);
  status = rppt_copy_gpu(image.mutable_gpu_data(), srcDescPtr,
                         blob->mutable_gpu_data(), dstDescPtr, handle);
  if (status != RPP_SUCCESS) {
    return false;
  }
  return true;
};

bool rppImageToGray(base::Image8UPtr &src, base::Image8UPtr &dst,
                    const int src_width, const int src_height,
                    const float coeffs[3]) {
  RppStatus status;

  RpptDesc srcDesc, dstDesc;
  RpptDescPtr srcDescPtr, dstDescPtr;
  srcDescPtr = &srcDesc;
  dstDescPtr = &dstDesc;

  rppInitDescriptor(srcDescPtr, src_width, src_height, 3, src->width_step());
  rppInitDescriptor(dstDescPtr, src_width, src_height, 3, dst->width_step());

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
  if (status != RPP_SUCCESS) {
    return false;
  }
  return true;
}

bool rppSwapImageChannels(base::Image8UPtr &src, base::Image8UPtr &dst,
                          const int src_width, const int src_height,
                          const int order[3]) {
  RppStatus status;
  RpptDesc srcDesc, dstDesc;
  RpptDescPtr srcDescPtr, dstDescPtr;
  srcDescPtr = &srcDesc;
  dstDescPtr = &dstDesc;

  rppInitDescriptor(srcDescPtr, src_width, src_height, 3, src->width_step());
  rppInitDescriptor(dstDescPtr, src_width, src_height, 3, dst->width_step());

  rppHandle_t handle;
  rppCreateWithBatchSize(&handle, 1);
  assert(order[0] == 2 && order[1] == 1 && order[2] == 0 &&
         "The order in rppt_swap_channels is hardcoded");
  status = rppt_swap_channels_gpu(src->mutable_gpu_data(), srcDescPtr,
                                  dst->mutable_gpu_data(), dstDescPtr, handle);
  if (status != RPP_SUCCESS) {
    return false;
  }
  return true;
}

bool rppDupImageChannels(base::Image8UPtr &src, base::Image8UPtr &dst,
                         const int src_width, const int src_height) {
  // TODO(B1tway) add a temporary implementation for Duplicate
  assert(0 && "Duplicate API has not yet been implemented in RPP");
  return false;
}
bool rppImageRemap(const base::Image8U &src_img, base::Image8U *dst_img,
                   const int src_width, const int src_height,
                   const base::Blob<float> &map_x,
                   const base::Blob<float> &map_y) {
  // TODO(B1tway) add a temporary implementation for Remap
  assert(0 && "Remap API has not yet been implemented in RPP");
  return false;
}

#endif
bool imageToBlob(base::Image8U &image, base::Blob<uint8_t> *blob) {
#if GPU_PLATFORM == NVIDIA
  return nppImageToBlob(image, blob);
#elif GPU_PLATFORM == AMD
  return rppImageToBlob(image, blob);
#endif
}

bool imageToGray(base::Image8UPtr &src, base::Image8UPtr &dst,
                 const int src_width, const int src_height,
                 const float coeffs[3]) {
#if GPU_PLATFORM == NVIDIA
  return nppImageToGray(src, dst, src_width, src_height, coeffs);
#elif GPU_PLATFORM == AMD
  return rppImageToGray(src, dst, src_width, src_height, coeffs);
#endif
}

bool swapImageChannels(base::Image8UPtr &src, base::Image8UPtr &dst,
                       const int src_width, const int src_height,
                       const int order[3]) {
#if GPU_PLATFORM == NVIDIA
  return nppSwapImageChannels(src, dst, src_width, src_height, order);
#elif GPU_PLATFORM == AMD
  return rppSwapImageChannels(src, dst, src_width, src_height, order);
#endif
}

bool dupImageChannels(base::Image8UPtr &src, base::Image8UPtr &dst,
                      const int src_width, const int src_height) {
#if GPU_PLATFORM == NVIDIA
  return nppDupImageChannels(src, dst, src_width, src_height);
#elif GPU_PLATFORM == AMD
  return rppDupImageChannels(src, dst, src_width, src_height);
#endif
}

bool imageRemap(const base::Image8U &src_img, base::Image8U *dst_img,
                const int src_width, const int src_height,
                const base::Blob<float> &map_x,
                const base::Blob<float> &map_y) {
#if GPU_PLATFORM == NVIDIA
  return nppImageRemap(src_img, dst_img, src_width, src_height, map_x, map_y);
#elif GPU_PLATFORM == AMD
  return rppImageRemap(src_img, dst_img, src_width, src_height, map_x, map_y);
#endif
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo