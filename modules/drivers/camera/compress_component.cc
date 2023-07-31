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


#ifdef __aarch64__
#include "NvBufSurface.h"
#else
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#endif

#include <iostream>
#include <fstream>


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

  width_ = config_.compress_conf().width();
  height_ = config_.compress_conf().height();

  writer_ = node_->CreateWriter<CompressedImage>(
      config_.compress_conf().output_channel());

#ifdef __aarch64__
  buffer_size_ = config_.compress_conf().width() *
    config_.compress_conf().height() * 3 / 2;
  try {
    nvbuffer_pool_.reset(new CCObjectPool<NvBuffer>(
        config_.compress_conf().image_pool_size()));
    nvbuffer_pool_->ConstructAll(
      V4L2_PIX_FMT_YUV420M, config_.compress_conf().width(),
      config_.compress_conf().height(), 0);
    for (size_t i = 0; i < config_.compress_conf().image_pool_size(); i++) {
      nvbuffer_pool_->GetObject()->allocateMemory();
    }
  } catch (const std::bad_alloc& e) {
    AERROR << e.what();
    return false;
  }

  try {
    downsampled_image_pool_.reset(
      new CCObjectPool<std::pair<std::vector<char>,
                                 std::vector<unsigned char>>>(
        config_.compress_conf().image_pool_size()));
    downsampled_image_pool_->ConstructAll();
    for (size_t i = 0; i < config_.compress_conf().image_pool_size(); i++) {
      (downsampled_image_pool_->GetObject()->first).reserve(buffer_size_);
      (downsampled_image_pool_->GetObject()->second).reserve(buffer_size_);
    }
  } catch (const std::bad_alloc& e) {
    AERROR << e.what();
    return false;
  }

  jpegenc_ = NvJPEGEncoder::createJPEGEncoder("jpegenc");
  jpegenc_->setCropRect(0, 0, 0, 0);
#endif

  return true;
}

bool CompressComponent::Proc(const std::shared_ptr<Image>& image) {
  ADEBUG << "procing compressed";
  if (cyber_unlikely(
      image->height() != height_ || image->width() != width_)) {
    AERROR << "Setting resolution is " << width_ << "x" << height_ <<
      ", but received " << image->width() << "x" << image->height();
    return false;
  }
  auto compressed_image = image_pool_->GetObject();
  compressed_image->mutable_header()->CopyFrom(image->header());
  compressed_image->set_frame_id(image->frame_id());
  compressed_image->set_measurement_time(image->measurement_time());
  compressed_image->set_height(height_);
  compressed_image->set_width(width_);

#ifdef __aarch64__
  compressed_image->set_format(image->encoding() + "; jpeg compressed");
  auto nvbuffer = nvbuffer_pool_->GetObject();
  auto downsampled_image = downsampled_image_pool_->GetObject();
  if (cyber_unlikely(YUV422ToYUV420(
      const_cast<char*>(image->data().data()),
      (downsampled_image->first).data()) != 0)) {
    AERROR << "Failed to downsample image";
    return false;
  }
  if (cyber_unlikely(ReadBuffer(
      (downsampled_image->first).data(), &(*nvbuffer)) != 0)) {
    AERROR << "Failed to convert image to nvbuffer";
    return false;
  }
  auto out_buffer = (downsampled_image->second).data();
  auto buffer_size = static_cast<uint64_t>(buffer_size_);

  auto ret = jpegenc_->encodeFromBuffer(*nvbuffer, JCS_YCbCr,
                            &out_buffer, buffer_size, 95);
  if (cyber_unlikely(ret != 0)) {
    AERROR << "nvjpeg failed to compress image";
  }
  compressed_image->set_data((downsampled_image->second).data(),
                              buffer_size);
#else
  compressed_image->set_format(image->encoding() + "; jpeg compressed bgr8");

  std::vector<int> params;
  params.resize(3, 0);
  params[0] = cv::IMWRITE_JPEG_QUALITY;
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
  } catch (std::exception& e) {
    AERROR << "cv::imencode (jpeg) exception :" << e.what();
    return false;
  }
#endif
  writer_->Write(compressed_image);
  return true;
}

#ifdef __aarch64__
int CompressComponent::YUV422ToYUV420(char* src, char* dst) {
  static uint line_length = width_ * 2;
  static uint y_num = width_ * height_;
  static uint cb_start = y_num;
  static uint cr_start = cb_start + cb_start / 4;
  uint i, j, k = 0;

  if (cyber_unlikely(!src || !dst))
    return -1;

  // Pick up y component
  for (i = 0; i < y_num; i++)
    dst[i] = src[i*2];

  // Interlaced scanning for picking up cb component
  for (i = 0; i < height_; i = i+2) {
    int offset = i*line_length;
    for (j = 1; j < line_length; j = j+4) {
      dst[cb_start+k] = src[offset+j];
      k++;
    }
  }

  // Interlaced scanning for picking up cr component
  k = 0;
  for (i = 1; i < height_; i = i+2) {
    int offset = i*line_length;
    for (j = 3; j < width_*2; j = j+4) {
      dst[cr_start+k] = src[offset+j];
      k++;
    }
  }

  return 0;
}

int CompressComponent::ReadBuffer(char* file_buffer, NvBuffer* buffer) {
  uint32_t i, j;
  char* data;

  for (i = 0; i < buffer->n_planes; i++) {
    NvBuffer::NvBufferPlane &plane = buffer->planes[i];
    int bytes_to_read =
      plane.fmt.bytesperpixel * plane.fmt.width;

    data = reinterpret_cast<char*>(plane.data);
    plane.bytesused = 0;
    for (j = 0; j < plane.fmt.height; j++) {
      memcpy(data, file_buffer, bytes_to_read);
      data += plane.fmt.stride;
      file_buffer += bytes_to_read;
    }
    plane.bytesused = plane.fmt.stride * plane.fmt.height;
  }
  return 0;
}
#endif

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
