/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/camera/compress_component.h"

#include <exception>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace apollo {
namespace drivers {
namespace camera {

bool CompressComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  AINFO << "Camera config: \n" << config_.DebugString();
  try {
    image_pool_.reset(new CCObjectPool<CompressedImage>(
        config_.compress_conf().image_pool_size()));
    image_pool_->ConstructAll();
  } catch (const std::bad_alloc& e) {
    AERROR << e.what();
    return false;
  }

  writer_ = node_->CreateWriter<CompressedImage>(
      config_.compress_conf().output_channel());
  return true;
}

bool CompressComponent::Proc(const std::shared_ptr<Image>& image) {
  ADEBUG << "procing compressed";
  auto compressed_image = image_pool_->GetObject();
  compressed_image->mutable_header()->CopyFrom(image->header());
  compressed_image->set_frame_id(image->frame_id());
  compressed_image->set_measurement_time(image->measurement_time());
  compressed_image->set_format(image->encoding() + "; jpeg compressed bgr8");

  std::vector<int> params;
  params.resize(3, 0);
  params[0] = CV_IMWRITE_JPEG_QUALITY;
  params[1] = 95;

  try {
    cv::Mat mat_image(image->height(), image->width(), CV_8UC3,
                      const_cast<char*>(image->data().data()), image->step());
    cv::Mat tmp_mat;
    cv::cvtColor(mat_image, tmp_mat, cv::COLOR_RGB2BGR);
    std::vector<uint8_t> compress_buffer;
    if (!cv::imencode(".jpg", tmp_mat, compress_buffer, params)) {
      AERROR << "cv::imencode (jpeg) failed on input image";
      return false;
    }
    compressed_image->set_data(compress_buffer.data(), compress_buffer.size());
    writer_->Write(compressed_image);
  } catch (std::exception& e) {
    AERROR << "cv::imencode (jpeg) exception :" << e.what();
    return false;
  }
  return true;
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
