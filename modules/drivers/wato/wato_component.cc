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

#include "modules/drivers/wato/wato_component.h"

namespace apollo {
namespace drivers {
namespace wato {

bool CameraComponent::Init() {
  wato_config_ = std::make_shared<Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               wato_config_.get())) {
    return false;
  }
  AINFO << "UsbCam config: " << wato_config_->DebugString();

  wato_device_.reset(new UsbCam());
  wato_device_->init(wato_config_);
  raw_image_.reset(new CameraImage);

  raw_image_->width = wato_config_->width();
  raw_image_->height = wato_config_->height();
  raw_image_->bytes_per_pixel = wato_config_->bytes_per_pixel();

  device_wait_ = wato_config_->device_wait_ms();
  spin_rate_ = static_cast<uint32_t>((1.0 / wato_config_->spin_rate()) * 1e6);

  if (wato_config_->output_type() == YUYV) {
    raw_image_->image_size = raw_image_->width * raw_image_->height * 2;
  } else if (wato_config_->output_type() == RGB) {
    raw_image_->image_size = raw_image_->width * raw_image_->height * 3;
  }
  if (raw_image_->image_size > MAX_IMAGE_SIZE) {
    AERROR << "image size is too big ,must less than " << MAX_IMAGE_SIZE
           << " bytes.";
    return false;
  }
  raw_image_->is_new = 0;
  // free memory in this struct desturctor
  raw_image_->image =
      reinterpret_cast<char*>(calloc(raw_image_->image_size, sizeof(char)));
  if (raw_image_->image == nullptr) {
    AERROR << "system calloc memory error, size:" << raw_image_->image_size;
    return false;
  }

  for (int i = 0; i < buffer_size_; ++i) {
    auto pb_image = std::make_shared<Image>();
    pb_image->mutable_header()->set_frame_id(wato_config_->frame_id());
    pb_image->set_width(raw_image_->width);
    pb_image->set_height(raw_image_->height);
    pb_image->mutable_data()->reserve(raw_image_->image_size);

    if (wato_config_->output_type() == YUYV) {
      pb_image->set_encoding("yuyv");
      pb_image->set_step(2 * raw_image_->width);
    } else if (wato_config_->output_type() == RGB) {
      pb_image->set_encoding("rgb8");
      pb_image->set_step(3 * raw_image_->width);
    }

    pb_image_buffer_.push_back(pb_image);
  }

  writer_ = node_->CreateWriter<Image>(wato_config_->channel_name());
  async_result_ = cyber::Async(&CameraComponent::run, this);
  return true;
}

void CameraComponent::run() {
  running_.exchange(true);
  while (!cyber::IsShutdown()) {
    if (!wato_device_->wait_for_device()) {
      // sleep for next check
      cyber::SleepFor(std::chrono::milliseconds(device_wait_));
      continue;
    }

    if (!wato_device_->poll(raw_image_)) {
      AERROR << "wato device poll failed";
      continue;
    }

    cyber::Time image_time(raw_image_->tv_sec, 1000 * raw_image_->tv_usec);
    if (index_ >= buffer_size_) {
      index_ = 0;
    }
    auto pb_image = pb_image_buffer_.at(index_++);
    pb_image->mutable_header()->set_timestamp_sec(
        cyber::Time::Now().ToSecond());
    pb_image->set_measurement_time(image_time.ToSecond());
    pb_image->set_data(raw_image_->image, raw_image_->image_size);
    writer_->Write(pb_image);

    cyber::SleepFor(std::chrono::microseconds(spin_rate_));
  }
}

CameraComponent::~CameraComponent() {
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

}  // namespace wato
}  // namespace drivers
}  // namespace apollo
