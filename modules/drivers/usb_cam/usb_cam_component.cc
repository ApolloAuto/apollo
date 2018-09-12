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

#include "modules/drivers/usb_cam/usb_cam_component.h"

namespace apollo {
namespace drivers {
namespace usb_cam {

bool UsbCamComponent::Init() {
  camera_config_ = std::make_shared<Config>();
  if(!apollo::cybertron::common::GetProtoFromFile(config_file_path_, camera_config_.get())){
    return false;
  }
  AINFO << "UsbCam config: " << camera_config_->DebugString();

  camera_device_.reset(new UsbCam());
  camera_device_->init(camera_config_);
  raw_image_.reset(new CameraImage);
  raw_image_->width = camera_config_->width();
  raw_image_->height = camera_config_->height();
  raw_image_->bytes_per_pixel = camera_config_->bytes_per_pixel();

  device_wait_ = camera_config_->device_wait();
  spin_rate_ = camera_config_->spin_rate();

  raw_image_->image_size =
      raw_image_->width * raw_image_->height * raw_image_->bytes_per_pixel;
  raw_image_->is_new = 0;
  // free memory in this struct desturctor
  raw_image_->image = (char*)calloc(raw_image_->image_size, sizeof(char));

  pb_image_.reset(new Image);
  pb_image_->mutable_header()->set_frame_id(camera_config_->frame_id());   
  pb_image_->set_encoding("yuyv");
  pb_image_->set_width(raw_image_->width);
  pb_image_->set_height(raw_image_->height);
  pb_image_->set_step(2 * raw_image_->width);
  pb_image_->mutable_data()->reserve(raw_image_->image_size);

  writer_ = node_->CreateWriter<Image>(camera_config_->channel_name());

  device_thread_ = std::shared_ptr<std::thread>(
    new std::thread(std::bind(&UsbCamComponent::run, this)));
  device_thread_->detach();
  return true;
}

void UsbCamComponent::run() {
  while (!cybertron::IsShutdown()) {
    if (!camera_device_->wait_for_device()) {
      //sleep 2s for next check
      sleep(device_wait_);
      continue;
    }

    if (!camera_device_->poll(raw_image_)) {
      LOG_ERROR << "camera device poll failed";
      continue;
    }
    
    cybertron::Time image_time(raw_image_->tv_sec, 1000 * raw_image_->tv_usec);
    pb_image_->mutable_header()->set_timestamp_sec(cybertron::Time::Now().ToSecond());   
    pb_image_->set_measurement_time(image_time.ToSecond());
    pb_image_->set_data(raw_image_->image, raw_image_->image_size);
    writer_->Write(pb_image_);

    sleep(spin_rate_);
  }
}

}
}
}  // namespace cybertron

