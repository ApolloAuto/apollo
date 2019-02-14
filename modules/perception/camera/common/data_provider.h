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

#include <nppi.h>

#include <memory>
#include <string>

#include "modules/perception/base/blob.h"
#include "modules/perception/base/box.h"
#include "modules/perception/base/image.h"
#include "modules/perception/camera/common/undistortion_handler.h"

namespace apollo {
namespace perception {
namespace camera {

class DataProvider {
 public:
  struct InitOptions {
    InitOptions()
        : image_height(0),
          image_width(0),
          device_id(-1),
          do_undistortion(false) {}

    int image_height;
    int image_width;
    int device_id;
    bool do_undistortion;
    std::string sensor_name;
  };

  struct ImageOptions {
    ImageOptions() {
      this->target_color = base::Color::NONE;
      this->do_crop = false;
    }

    ImageOptions(base::Color target_color, bool do_crop, base::RectI crop_roi) {
      this->target_color = target_color;
      this->do_crop = do_crop;
      this->crop_roi = crop_roi;
    }

    std::string ToString() {
      std::stringstream ss;
      ss << " " << static_cast<int>(target_color);
      ss << " " << do_crop;
      if (do_crop) {
        ss << " " << crop_roi.x << " " << crop_roi.y << " " << crop_roi.width
           << " " << crop_roi.height;
      }
      return ss.str();
    }

    base::Color target_color = base::Color::BGR;
    bool do_crop = false;  // default: DONOT crop
    base::RectI crop_roi;
  };

  DataProvider() = default;
  ~DataProvider() = default;

  DataProvider(const DataProvider &) = delete;
  DataProvider &operator=(const DataProvider &) = delete;

  bool Init(const InitOptions &options = InitOptions());

  // @brief: fill raw image data.
  // @param [in]: options
  // @param [in/out]: blob
  // image blob with specified size should be filled, required.
  bool FillImageData(int rows, int cols, const uint8_t *data,
                     const std::string &encoding);

#if 0
  // @brief: get blob converted from raw message.
  // @param [in]: options
  // @param [in/out]: NHWC blob (4D)
  // image blob with specified size should be filled, required.
  bool GetImageBlob(const ImageOptions &options, base::Blob<float> *blob);
#endif

  // @brief: get blob converted from raw message.
  // @param [in]: options
  // @param [in/out]: NHWC blob (4D)
  // image blob with specified size should be filled, required.
  bool GetImageBlob(const ImageOptions &options, base::Blob<uint8_t> *blob);

  // @brief: get Image8U converted from raw message.
  // @param [in]: options
  // @return: Image8U
  // image blob with specified size should be filled, required.
  bool GetImage(const ImageOptions &options, base::Image8U *image);

  int src_height() const { return src_height_; }
  int src_width() const { return src_width_; }
  const std::string &sensor_name() const { return sensor_name_; }

  bool to_gray_image();
  bool to_rgb_image();
  bool to_bgr_image();

 protected:
  std::string sensor_name_;
  int src_height_ = 0;
  int src_width_ = 0;
  int device_id_ = -1;

  std::shared_ptr<base::Image8U> ori_gray_;
  std::shared_ptr<base::Image8U> ori_rgb_;
  std::shared_ptr<base::Image8U> ori_bgr_;
  std::shared_ptr<base::Image8U> gray_;
  std::shared_ptr<base::Image8U> rgb_;
  std::shared_ptr<base::Image8U> bgr_;
  bool gray_ready_ = false;
  bool rgb_ready_ = false;
  bool bgr_ready_ = false;

  base::Blob<float> temp_float_;
  base::Blob<uint8_t> temp_uint8_;
  std::shared_ptr<UndistortionHandler> handler_ = nullptr;
};  // class DataProvider

}  // namespace camera
}  // namespace perception
}  // namespace apollo
