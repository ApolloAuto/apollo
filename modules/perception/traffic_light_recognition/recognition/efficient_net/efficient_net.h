/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "cyber/common/macros.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/traffic_light_recognition/interface/base_traffic_light_recognitor.h"
#include "modules/perception/traffic_light_recognition/recognition/efficient_net/proto/efficient_net_param.pb.h"

namespace apollo {
namespace perception {
namespace trafficlight {

class EfficientNetRecognition final : public BaseTrafficLightRecognitor {
public:
    /**
     * @brief Construct a new Traffic Light Recognition object
     *
     */
    EfficientNetRecognition() = default;
    /**
     * @brief Destroy the Traffic Light Recognition object
     *
     */
    ~EfficientNetRecognition() = default;
    /**
     * @brief Initialize traffic light recognitor parameters.
     *
     * @param options
     * @return true
     * @return false
     */
    bool Init(const TrafficLightRecognitorInitOptions& options) override;

    /**
     * @brief recogn traffic_light from image.
     *
     * @param frame
     * @return true
     * @return false
     */
    bool Detect(camera::TrafficLightFrame* frame) override;

private:
    bool InitModel();
    void Perform(const camera::TrafficLightFrame* frame, std::vector<base::TrafficLightPtr>* lights);
    void ZeroPadding(
            const base::RectI& light_roi,
            base::Image8U* roi_tmp_image,
            int* dst_width,
            int* dst_height,
            const int& provider_w,
            const int& provider_h);
    void Prob2Color(const float* output_data, float threshold, base::TrafficLightPtr light);

private:
    // input/output blobs
    std::shared_ptr<base::Blob<float>> input_blob_;
    std::shared_ptr<base::Blob<float>> outputs_cls_;
    std::shared_ptr<base::Blob<float>> outputs_status_;
    // image
    std::shared_ptr<base::Image8U> image_ = nullptr;
    std::shared_ptr<base::Blob<float>> mean_buffer_;
    std::shared_ptr<base::Blob<float>> mean_;
    camera::DataProvider::ImageOptions data_provider_image_option_;
    // gpu id
    int gpu_id_;
    cudaStream_t stream_ = nullptr;
    EfficientNetParam model_param_;
    std::shared_ptr<inference::Inference> inference_;

    // image shape
    int resize_height_;
    int resize_width_;
    float unknown_threshold_;
    float scale_;

    DISALLOW_COPY_AND_ASSIGN(EfficientNetRecognition);
};

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
