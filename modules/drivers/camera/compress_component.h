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

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"

#ifdef __aarch64__
#include "NvJpegEncoder.h"
#include "NvBuffer.h"
#include "NvUtils.h"
#endif

namespace apollo {
namespace drivers {
namespace camera {

class CompressComponent : public apollo::cyber::Component<
                                    apollo::drivers::Image> {
public:
    bool Init() override;
    bool Proc(
        const std::shared_ptr<apollo::drivers::Image>& image) override;

    ~CompressComponent() {}

private:
    // std::shared_ptr<
    //     apollo::cyber::base::CCObjectPool<CompressedImage>> image_pool_;
    std::shared_ptr<
        apollo::cyber::Writer<CompressedImage>> writer_ = nullptr;
    apollo::drivers::camera::config::Config config_;
    uint width_;
    uint height_;

#ifdef __aarch64__
    int YUV422ToYUV420(char* src, char* dst);
    int ReadBuffer(char* file_buffer, NvBuffer* buffer);

    NvJPEGEncoder* jpegenc_;
    int buffer_size_;
    std::shared_ptr<apollo::cyber::base::CCObjectPool<NvBuffer>> nvbuffer_pool_;
    std::shared_ptr<apollo::cyber::base::CCObjectPool<std::pair<std::vector<char>, std::vector<unsigned char>>>> downsampled_image_pool_;
#endif
};

CYBER_REGISTER_COMPONENT(CompressComponent)
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
