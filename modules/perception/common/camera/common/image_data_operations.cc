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
#include "modules/perception/common/camera/common/image_data_operations.h"

#include "cyber/common/log.h"

#if GPU_PLATFORM == NVIDIA
#include <nppi.h>

#include "modules/perception/common/camera/common/image_data_operations_npp.h"
#elif GPU_PLATFORM == AMD
#include <rppi.h>

#include "modules/perception/common/camera/common/image_data_operations_rpp.h"
#define nppImageToBlob rppImageToBlob
#define nppImageToGray rppImageToGray
#define nppImageRemap rppImageRemap
#define nppSwapImageChannels rppSwapImageChannels
#define nppDupImageChannels rppDupImageChannels
#endif

namespace apollo {
namespace perception {
namespace camera {

bool imageToBlob(const base::Image8U &image, base::Blob<uint8_t> *blob) {
  return nppImageToBlob(image, blob);
}

bool imageToGray(const base::Image8UPtr &src, const base::Image8UPtr &dst,
                 const int src_width, const int src_height,
                 const float coeffs[3]) {
  return nppImageToGray(src, dst, src_width, src_height, coeffs);
}

bool swapImageChannels(const base::Image8UPtr &src, const base::Image8UPtr &dst,
                       const int src_width, const int src_height,
                       const int order[3]) {
  return nppSwapImageChannels(src, dst, src_width, src_height, order);
}

bool dupImageChannels(const base::Image8UPtr &src, const base::Image8UPtr &dst,
                      const int src_width, const int src_height) {
  return nppDupImageChannels(src, dst, src_width, src_height);
}

bool imageRemap(const base::Image8U &src_img, base::Image8U *dst_img,
                const int src_width, const int src_height,
                const base::Blob<float> &map_x,
                const base::Blob<float> &map_y) {
  return nppImageRemap(src_img, dst_img, src_width, src_height, map_x, map_y);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
