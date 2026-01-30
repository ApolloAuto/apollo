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

#include "modules/drivers/camera/camera_component.h"
#ifdef __aarch64__
#include <cuda_runtime.h>
#endif

namespace apollo {
namespace drivers {
namespace camera {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::Image;
using apollo::drivers::camera::config::Config;
using apollo::drivers::camera::config::RGB;
using apollo::drivers::camera::config::YUYV;

bool CameraComponent::Init() {
    // hook: Apollo License Verification: v_apollo_park
    camera_config_ = std::make_shared<Config>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, camera_config_.get())) {
        return false;
    }
    AINFO << "UsbCam config: " << camera_config_->DebugString();

    camera_device_.reset(new UsbCam());
    camera_device_->init(camera_config_);
    raw_image_.reset(new CameraImage);
    raw_image_for_compress_.reset(new CameraImage);

    raw_image_->width = camera_config_->width();
    raw_image_->height = camera_config_->height();
    raw_image_->bytes_per_pixel = camera_config_->bytes_per_pixel();

    raw_image_for_compress_->width = camera_config_->width();
    raw_image_for_compress_->height = camera_config_->height();
    raw_image_for_compress_->bytes_per_pixel = camera_config_->bytes_per_pixel();

    if (camera_config_->pixel_format() == "yuyv" || camera_config_->pixel_format() == "uyvy"
        || camera_config_->pixel_format() == "yuvmono10") {
        raw_image_for_compress_->image_size = raw_image_for_compress_->width * raw_image_for_compress_->height * 2;
    } else if (camera_config_->pixel_format() == "mjpeg") {
        AINFO << "Disable sensor raw camera output with format mjpeg";
        raw_image_for_compress_->image_size = 0;
    } else if (camera_config_->pixel_format() == "rgb24") {
        raw_image_for_compress_->image_size = raw_image_for_compress_->width * raw_image_for_compress_->height * 3;
    } else {
        AERROR << "Wrong pixel fromat:" << camera_config_->pixel_format()
               << ",must be yuyv | uyvy | mjpeg | yuvmono10 | rgb24";
        return false;
    }

    if (raw_image_for_compress_->image_size == 0) {
        raw_image_for_compress_->image = nullptr;
    } else {
        raw_image_for_compress_->image
                = reinterpret_cast<char*>(calloc(raw_image_for_compress_->image_size, sizeof(char)));
    }

    device_wait_ = camera_config_->device_wait_ms();
    spin_rate_ = static_cast<uint32_t>((1.0 / camera_config_->spin_rate()) * 1e6);

    if (camera_config_->output_type() == YUYV) {
        raw_image_->image_size = raw_image_->width * raw_image_->height * 2;
    } else if (camera_config_->output_type() == RGB) {
        raw_image_->image_size = raw_image_->width * raw_image_->height * 3;
    }
    if (raw_image_->image_size > MAX_IMAGE_SIZE) {
        AERROR << "image size is too big ,must less than " << MAX_IMAGE_SIZE << " bytes.";
        return false;
    }
    raw_image_->is_new = 0;
    // free memory in this struct desturctor
#ifndef __aarch64__
    raw_image_->image = reinterpret_cast<char*>(calloc(raw_image_->image_size, sizeof(char)));
    if (raw_image_->image == nullptr) {
        AERROR << "system calloc memory error, size:" << raw_image_->image_size;
        return false;
    }
#else
    if (camera_config_->arm_gpu_acceleration()) {
        if (cudaMallocHost(&raw_image_->image, raw_image_->image_size * sizeof(char), cudaHostAllocMapped)
            != cudaSuccess) {
            AERROR << "cuda malloc memory error, size:" << raw_image_->image_size;
            return false;
        }
    } else {
        raw_image_->image = reinterpret_cast<char*>(calloc(raw_image_->image_size, sizeof(char)));
        if (raw_image_->image == nullptr) {
            AERROR << "system calloc memory error, size:" << raw_image_->image_size;
            return false;
        }
    }
#endif

    writer_ = node_->CreateWriter<Image>(camera_config_->channel_name());
    raw_writer_ = node_->CreateWriter<Image>(camera_config_->raw_channel_name());

    async_result_ = cyber::Async(&CameraComponent::run, this);
    return true;
}

void CameraComponent::run() {
    running_.exchange(true);
    while (!cyber::IsShutdown()) {
        if (!camera_device_->wait_for_device()) {
            // sleep for next check
            cyber::SleepFor(std::chrono::milliseconds(device_wait_));
            continue;
        }

        auto s = camera_device_->poll(raw_image_, raw_image_for_compress_);
        if (s != CameraDeviceState::STATE_SUCCESS) {
            if (s == CameraDeviceState::STATE_DROP) {
                continue;
            } else {
                AERROR << "camera device poll failed";
                continue;
            }
        }

        cyber::Time image_time(raw_image_->tv_sec, 1000 * raw_image_->tv_usec);
        if (index_ >= buffer_size_) {
            index_ = 0;
        }
        auto pb_image = writer_->AcquireMessage();
        pb_image->mutable_header()->set_frame_id(camera_config_->frame_id());
        pb_image->set_width(raw_image_->width);
        pb_image->set_height(raw_image_->height);
        pb_image->mutable_data()->reserve(raw_image_->image_size);

        if cyber_likely(camera_config_->output_type() == YUYV) {
            pb_image->set_encoding("yuyv");
            pb_image->set_step(2 * raw_image_->width);
        } else if (camera_config_->output_type() == RGB) {
            pb_image->set_encoding("rgb8");
            pb_image->set_step(3 * raw_image_->width);
        }
        auto header_time = cyber::Time::Now().ToSecond();
        auto measurement_time = image_time.ToSecond();
        if (measurement_time > header_time) {
            // measurement_time should never greater than header_time
            // invalid v4l2 buffer time, use header_time instead
            measurement_time = header_time; 
        }
        pb_image->mutable_header()->set_timestamp_sec(header_time);
        pb_image->set_measurement_time(measurement_time);
        pb_image->set_data(raw_image_->image, raw_image_->image_size);
        writer_->Write(pb_image);

        auto raw_image_for_compress = raw_writer_->AcquireMessage();
        raw_image_for_compress->mutable_header()->set_frame_id(camera_config_->frame_id());
        raw_image_for_compress->set_width(raw_image_for_compress_->width);
        raw_image_for_compress->set_height(raw_image_for_compress_->height);
        raw_image_for_compress->mutable_data()->reserve(raw_image_for_compress_->image_size);
        raw_image_for_compress->set_encoding(camera_config_->pixel_format());
        raw_image_for_compress->mutable_header()->set_timestamp_sec(header_time);
        raw_image_for_compress->set_measurement_time(measurement_time);
        raw_image_for_compress->set_data(raw_image_for_compress_->image, raw_image_for_compress_->image_size);
        raw_writer_->Write(raw_image_for_compress);

#ifndef __aarch64__
        // the frame rate become unstable if yielding cpu in aarch64
        // thus only yielding cpu in x86_64
        cyber::SleepFor(std::chrono::microseconds(spin_rate_));
#endif
    }
}

CameraComponent::~CameraComponent() {
    if (running_.load()) {
        running_.exchange(false);
#ifndef __aarch64__
        free(raw_image_->image);
#else
        cudaFreeHost(raw_image_->image);
#endif
        async_result_.wait();
    }
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
