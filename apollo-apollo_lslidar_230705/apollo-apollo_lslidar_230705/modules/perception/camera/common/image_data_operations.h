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

#include "modules/perception/base/blob.h"
#include "modules/perception/base/image.h"

namespace apollo {
namespace perception {
namespace camera {

bool imageToBlob(const base::Image8U &image, base::Blob<uint8_t> *blob);

bool imageToGray(const base::Image8UPtr &src,
                 const base::Image8UPtr &dst,
                 const int src_width, const int src_height,
                 const float coeffs[3]);

bool swapImageChannels(const base::Image8UPtr &src,
                       const base::Image8UPtr &dst,
                       const int src_width, const int src_height,
                       const int order[3]);

bool dupImageChannels(const base::Image8UPtr &src,
                      const base::Image8UPtr &dst,
                      const int src_width, const int src_height);

bool imageRemap(const base::Image8U &src_img, base::Image8U *dst_img,
                const int src_width, const int src_height,
                const base::Blob<float> &map_x,
                const base::Blob<float> &map_y);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
