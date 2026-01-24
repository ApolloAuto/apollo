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
#include "modules/perception/traffic_light_detection/algorithm/cropbox.h"

#include <algorithm>

#include "modules/perception/common/camera/common/util.h"

namespace apollo {
namespace perception {
namespace trafficlight {

void CropBox::getCropBox(const int width, const int height, const base::TrafficLightPtr &light, base::RectI *crop_box) {
    int rows = height;
    int cols = width;

    if (camera::OutOfValidRegion(light->region.projection_roi, width, height)
        || light->region.projection_roi.Area() <= 0) {
        crop_box->x = 0;
        crop_box->y = 0;
        crop_box->width = 0;
        crop_box->height = 0;
        return;
    }

    int xl = light->region.projection_roi.x;
    int yt = light->region.projection_roi.y;
    int xr = xl + light->region.projection_roi.width - 1;
    int yb = yt + light->region.projection_roi.height - 1;

    // scale
    int center_x = (xr + xl) / 2;
    int center_y = (yb + yt) / 2;
    int resize = static_cast<int>(
            crop_scale_
            * static_cast<float>(std::max(light->region.projection_roi.width, light->region.projection_roi.height)));

    resize = std::max(resize, min_crop_size_);
    resize = std::min(resize, width);
    resize = std::min(resize, height);

    // clamp
    xl = center_x - resize / 2 + 1;
    xl = (xl < 0) ? 0 : xl;
    yt = center_y - resize / 2 + 1;
    yt = (yt < 0) ? 0 : yt;
    xr = xl + resize - 1;
    yb = yt + resize - 1;
    if (xr >= cols - 1) {
        xl -= xr - cols + 1;
        xr = cols - 1;
    }

    if (yb >= rows - 1) {
        yt -= yb - rows + 1;
        yb = rows - 1;
    }

    crop_box->x = xl;
    crop_box->y = yt;
    crop_box->width = xr - xl + 1;
    crop_box->height = yb - yt + 1;
}
void CropBox::Init(float crop_scale, int min_crop_size) {
    crop_scale_ = crop_scale;
    min_crop_size_ = min_crop_size;
}
CropBox::CropBox(float crop_scale, int min_crop_size) {
    Init(crop_scale, min_crop_size);
}
void CropBoxWholeImage::getCropBox(
        const int width,
        const int height,
        const base::TrafficLightPtr &light,
        base::RectI *crop_box) {
    if (!camera::OutOfValidRegion(light->region.projection_roi, width, height)
        && light->region.projection_roi.Area() > 0) {
        crop_box->x = crop_box->y = 0;
        crop_box->width = width;
        crop_box->height = height;
        return;
    }

    crop_box->x = 0;
    crop_box->y = 0;
    crop_box->width = 0;
    crop_box->height = 0;
}

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
