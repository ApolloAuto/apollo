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
#include "modules/perception/camera/common/data_provider.h"

#include "cyber/common/log.h"
#include "modules/perception/camera/common/image_data_operations.h"

namespace apollo {
namespace perception {
namespace camera {

bool DataProvider::Init(const DataProvider::InitOptions &options) {
  src_height_ = options.image_height;
  src_width_ = options.image_width;
  sensor_name_ = options.sensor_name;
  device_id_ = options.device_id;

  if (cudaSetDevice(device_id_) != cudaSuccess) {
    AERROR << "Failed to set device to: " << device_id_;
    return false;
  }

  // Initialize uint8 blobs
  gray_.reset(new base::Image8U(src_height_, src_width_, base::Color::GRAY));
  rgb_.reset(new base::Image8U(src_height_, src_width_, base::Color::RGB));
  bgr_.reset(new base::Image8U(src_height_, src_width_, base::Color::BGR));

  // Allocate CPU memory for uint8 blobs
  gray_->cpu_data();
  rgb_->cpu_data();
  bgr_->cpu_data();

  // Allocate GPU memory for uint8 blobs
  gray_->gpu_data();
  rgb_->gpu_data();
  bgr_->gpu_data();

  if (options.do_undistortion) {
    handler_.reset(new UndistortionHandler());
    if (!handler_->Init(options.sensor_name, device_id_)) {
      return false;
    }
    // Initialize uint8 blobs
    ori_gray_.reset(
        new base::Image8U(src_height_, src_width_, base::Color::GRAY));
    ori_rgb_.reset(
        new base::Image8U(src_height_, src_width_, base::Color::RGB));
    ori_bgr_.reset(
        new base::Image8U(src_height_, src_width_, base::Color::BGR));

    // Allocate CPU memory for uint8 blobs
    ori_gray_->cpu_data();
    ori_rgb_->cpu_data();
    ori_bgr_->cpu_data();

    // Allocate GPU memory for uint8 blobs
    ori_gray_->gpu_data();
    ori_rgb_->gpu_data();
    ori_bgr_->gpu_data();
  }

  // Warm up nppi functions
  {
    bgr_ready_ = false;
    rgb_ready_ = true;
    gray_ready_ = false;
    to_bgr_image();
    bgr_ready_ = false;
    rgb_ready_ = false;
    gray_ready_ = true;
    to_bgr_image();
  }
  {
    bgr_ready_ = false;
    rgb_ready_ = false;
    gray_ready_ = true;
    to_rgb_image();
    bgr_ready_ = true;
    rgb_ready_ = false;
    gray_ready_ = false;
    to_rgb_image();
  }
  {
    bgr_ready_ = false;
    rgb_ready_ = true;
    gray_ready_ = false;
    to_gray_image();
    bgr_ready_ = true;
    rgb_ready_ = false;
    gray_ready_ = false;
    to_gray_image();
  }
  bgr_ready_ = false;
  rgb_ready_ = false;
  gray_ready_ = false;

  return true;
}

bool DataProvider::FillImageData(int rows, int cols, const uint8_t *data,
                                 const std::string &encoding) {
  if (cudaSetDevice(device_id_) != cudaSuccess) {
    AERROR << "Failed to set device to: " << device_id_;
    return false;
  }

  gray_ready_ = false;
  rgb_ready_ = false;
  bgr_ready_ = false;

  bool success = false;

#if USE_GPU == 0  // copy to host memory
  AINFO << "Fill in CPU mode ...";
  if (handler_ != nullptr) {
    AERROR << "Undistortion DO NOT support CPU mode!";
    return false;
  }
  if (encoding == "rgb8") {
    memcpy(rgb_->mutable_cpu_data(), data, rgb_->count() * sizeof(data[0]));
    rgb_ready_ = true;
    success = true;
  } else if (encoding == "bgr8") {
    memcpy(bgr_->mutable_cpu_data(), data, bgr_->count() * sizeof(data[0]));
    bgr_ready_ = true;
    success = true;
  } else if (encoding == "gray" || encoding == "y") {
    memcpy(gray_->mutable_cpu_data(), data, gray_->count() * sizeof(data[0]));
    gray_ready_ = true;
    success = true;
  } else {
    AERROR << "Unrecognized image encoding: " << encoding;
  }
#else  // copy to device memory directly
  AINFO << "Fill in GPU mode ...";
  if (encoding == "rgb8") {
    if (handler_ != nullptr) {
      cudaMemcpy(ori_rgb_->mutable_gpu_data(), data,
                 ori_rgb_->rows() * ori_rgb_->width_step(), cudaMemcpyDefault);
      success = handler_->Handle(*ori_rgb_, rgb_.get());
    } else {
      cudaMemcpy(rgb_->mutable_gpu_data(), data,
                 rgb_->rows() * rgb_->width_step(), cudaMemcpyDefault);
      success = true;
    }
    rgb_ready_ = true;
  } else if (encoding == "bgr8") {
    if (handler_ != nullptr) {
      cudaMemcpy(ori_bgr_->mutable_gpu_data(), data,
                 ori_bgr_->rows() * ori_bgr_->width_step(), cudaMemcpyDefault);
      success = handler_->Handle(*ori_bgr_, bgr_.get());
    } else {
      cudaMemcpy(bgr_->mutable_gpu_data(), data,
                 bgr_->rows() * bgr_->width_step(), cudaMemcpyDefault);
      success = true;
    }
    bgr_ready_ = true;
  } else if (encoding == "gray" || encoding == "y") {
    if (handler_ != nullptr) {
      cudaMemcpy(ori_gray_->mutable_gpu_data(), data,
                 ori_gray_->rows() * ori_gray_->width_step(),
                 cudaMemcpyDefault);
      success = handler_->Handle(*ori_gray_, gray_.get());
    } else {
      cudaMemcpy(gray_->mutable_gpu_data(), data,
                 gray_->rows() * gray_->width_step(), cudaMemcpyDefault);
      success = true;
    }
    gray_ready_ = true;
  } else {
    AERROR << "Unrecognized image encoding: " << encoding;
  }
#endif

  AINFO << "Done! (" << success << ")";
  return success;
}

#if 0
bool DataProvider::GetImageBlob(const DataProvider::ImageOptions &options,
                                base::Blob<float> *blob) {
  bool ret = GetImageBlob(options, &temp_uint8_);
  if (!ret) {
    return false;
  }
  blob->Reshape(temp_uint8_.shape());
  NppiSize roi;
  roi.height = temp_uint8_.shape(1);
  roi.width = temp_uint8_.shape(2);
  int channels = base::kChannelsMap.at(options.target_color);
  const uint8_t *temp_ptr = temp_uint8_.gpu_data();
  float *blob_ptr = blob->mutable_gpu_data();
  int temp_step = temp_uint8_.count(2) * sizeof(uint8_t);
  int blob_step = blob->count(2) * sizeof(float);
#if GPU_PLATFORM == NVIDIA
  if (channels == 1) {
    nppiConvert_8u32f_C1R(temp_ptr, temp_step, blob_ptr, blob_step, roi);
  } else {
    nppiConvert_8u32f_C3R(temp_ptr, temp_step, blob_ptr, blob_step, roi);
  }
#elif GPU_PLATFORM == AMD
    // TODO(B1tway): Add necesssary RPP API
#endif
  return true;
}
#endif

bool DataProvider::GetImageBlob(const DataProvider::ImageOptions &options,
                                base::Blob<uint8_t> *blob) {
  base::Image8U image;
  if (!GetImage(options, &image)) {
    return false;
  }
  imageToBlob(image, blob);
  return true;
}

bool DataProvider::GetImage(const DataProvider::ImageOptions &options,
                            base::Image8U *image) {
  AINFO << "GetImage ...";
  if (image == nullptr) {
    return false;
  }
  bool success = false;
  switch (options.target_color) {
    case base::Color::RGB:
      success = to_rgb_image();
      *image = (*rgb_);
      break;
    case base::Color::BGR:
      success = to_bgr_image();
      *image = (*bgr_);
      break;
    case base::Color::GRAY:
      success = to_gray_image();
      *image = (*gray_);
      break;
    default:
      AERROR << "Unsupported Color: "
             << static_cast<uint8_t>(options.target_color);
  }
  if (!success) {
    return false;
  }

  if (options.do_crop) {
    AINFO << "\tcropping ...";
    *image = (*image)(options.crop_roi);
  }
  AINFO << "Done!";
  return true;
}

bool DataProvider::to_gray_image() {
  if (!gray_ready_) {
    if (bgr_ready_) {
      float coeffs[] = {0.114f, 0.587f, 0.299f};
      imageToGray(bgr_, gray_, src_width_, src_height_, coeffs);
      gray_ready_ = true;
    } else if (rgb_ready_) {
      float coeffs[] = {0.299f, 0.587f, 0.114f};
      imageToGray(rgb_, gray_, src_width_, src_height_, coeffs);
      gray_ready_ = true;
    } else {
      AWARN << "No image data filled yet, return uninitialized blob!";
      return false;
    }
  }
  return true;
}

bool DataProvider::to_rgb_image() {
  if (!rgb_ready_) {
    if (bgr_ready_) {
      // BGR2RGB takes less than 0.010ms on K2200
      const int order[] = {2, 1, 0};
      swapImageChannels(bgr_, rgb_, src_width_, src_height_, order);
      rgb_ready_ = true;
    } else if (gray_ready_) {
      dupImageChannels(gray_, rgb_, src_width_, src_height_);
      rgb_ready_ = true;
    } else {
      AWARN << "No image data filled yet, return uninitialized blob!";
      return false;
    }
  }
  return true;
}

bool DataProvider::to_bgr_image() {
  if (!bgr_ready_) {
    if (rgb_ready_) {
      const int order[] = {2, 1, 0};
      swapImageChannels(rgb_, bgr_, src_width_, src_height_, order);
      bgr_ready_ = true;
    } else if (gray_ready_) {
      dupImageChannels(gray_, bgr_, src_width_, src_height_);
      bgr_ready_ = true;
    } else {
      AWARN << "No image data filled yet, return uninitialized blob!";
      return false;
    }
  }
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
