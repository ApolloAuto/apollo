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

#include "modules/drivers/tools/image_decompress/image_decompress.h"

#include <vector>

#include "opencv2/opencv.hpp"

namespace apollo {
namespace image_decompress {

using apollo::drivers::Image;

bool ImageDecompressComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  AINFO << "Decompress config: \n" << config_.DebugString();
  writer_ = node_->CreateWriter<Image>(config_.channel_name());
  return true;
}

bool ImageDecompressComponent::Proc(
    const std::shared_ptr<apollo::drivers::CompressedImage>& compressed_image) {
  auto image = std::make_shared<Image>();
  image->mutable_header()->CopyFrom(compressed_image->header());
  if (compressed_image->has_measurement_time()) {
    image->set_measurement_time(compressed_image->measurement_time());
  } else {
    image->set_measurement_time(compressed_image->header().timestamp_sec());
  }
  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                           compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, cv::IMREAD_COLOR);
  cv::cvtColor(mat_image, mat_image, cv::COLOR_BGR2RGB);
  image->set_width(mat_image.cols);
  image->set_height(mat_image.rows);
  // Now olny rgb
  image->set_encoding("rgb8");
  image->set_step(3 * image->width());

  auto size = mat_image.step * mat_image.rows;
  image->set_data(&(mat_image.data[0]), size);
  writer_->Write(image);
  return true;
}

}  // namespace image_decompress
}  // namespace apollo
