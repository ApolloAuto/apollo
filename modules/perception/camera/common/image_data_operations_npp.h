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
#pragma once

namespace apollo {
namespace perception {
namespace camera {

bool nppImageToBlob(const base::Image8U &image, base::Blob<uint8_t> *blob) {
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

bool nppImageToGray(const base::Image8UPtr &src, const base::Image8UPtr &dst,
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

bool nppSwapImageChannels(const base::Image8UPtr &src,
                          const base::Image8UPtr &dst,
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

bool nppDupImageChannels(const base::Image8UPtr &src,
                         const base::Image8UPtr &dst,
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
}  // namespace camera
}  // namespace perception
}  // namespace apollo
