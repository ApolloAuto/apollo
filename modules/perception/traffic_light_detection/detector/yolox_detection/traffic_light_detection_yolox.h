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

#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "modules/perception/traffic_light_detection/detector/yolox_detection/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/image_8u.h"
#include "modules/perception/common/base/traffic_light.h"
#include "modules/perception/common/camera/common/timer.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/traffic_light_detection/algorithm/cropbox.h"
#include "modules/perception/traffic_light_detection/algorithm/select.h"
#include "modules/perception/traffic_light_detection/interface/base_traffic_light_detector.h"

namespace apollo {
namespace perception {
namespace trafficlight {

struct PadResizeParam {
    int left_pad = 0;
    int right_pad = 0;
    int top_pad = 0;
    int bottom_pad = 0;
    int roi_width = 0;
    int roi_height = 0;
    int roi_x = 0;
    int roi_y = 0;
    float scale = 1.f;
    PadResizeParam(
            int l_pad = 0,
            int r_pad = 0,
            int t_pad = 0,
            int b_pad = 0,
            int r_w = 0,
            int r_h = 0,
            float s = 1.f,
            int r_x = 0,
            int r_y = 0) {
        left_pad = l_pad;
        right_pad = r_pad;
        top_pad = t_pad;
        bottom_pad = b_pad;
        roi_width = r_w;
        roi_height = r_h;
        scale = s;
        roi_x = r_x;
        roi_y = r_y;
    }
    void Reset() {
        left_pad = 0;
        right_pad = 0;
        top_pad = 0;
        bottom_pad = 0;
        roi_width = 0;
        roi_height = 0;
        scale = 1.f;
        roi_x = 0;
        roi_y = 0;
    }
};

class TrafficLightTLDetectorYolox : public BaseTrafficLightDetector {
public:
    TrafficLightTLDetectorYolox() = default;

    ~TrafficLightTLDetectorYolox() = default;

    bool Init(const TrafficLightDetectorInitOptions& options) override;

    bool Detect(camera::TrafficLightFrame* frame) override;

    bool Inference(camera::TrafficLightFrame* frame);

    const std::vector<base::TrafficLightPtr>& getDetectedBoxes() {
        return detected_bboxes_;
    }

private:
    bool DetectHeadByModel(camera::TrafficLightFrame* frame);

    // work function
    bool DetectTL(camera::TrafficLightFrame* frame);

    bool GetCandidateHeads(
            const int batch_id,
            std::vector<base::TrafficLightPtr>* lights,
            std::vector<base::TrafficLightPtr>& lights_ref);

    void AddShapeYolox(
            std::map<std::string, std::vector<int>>* shape_map,
            const google::protobuf::RepeatedPtrField<common::ModelBlob>& model_blobs);
    void ApplyNMS(std::vector<base::TrafficLightPtr>* lights);
    void ApplyOverlapNMS(std::vector<base::TrafficLightPtr>* lights);
    void NMS(std::vector<base::TrafficLightPtr>* lights, const std::string camera_name);

private:
    std::shared_ptr<inference::Inference> detect_net_ = nullptr;
    std::map<std::string, std::vector<base::TrafficLightPtr>> detected_heads_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    float cls_th_ = 0.01;
    int resize_height_ = 640;
    int resize_width_ = 640;
    // vertical_light， quadrate_light， horizontal_light
    int tl_shape_classes_ = 3;
    int min_head_area_ = 10;
    int max_batch_roi_ = 3000;

    camera::DataProvider::ImageOptions data_provider_image_option_;
    // traffic light detection params
    trafficlight::yolox::ModelParam detection_param_;
    std::vector<base::TrafficLightPtr> detected_bboxes_;
    std::vector<std::string> bbox_project_camera_;
    std::vector<std::string> detected_light_ids_;
    int max_batch_size_ = 4;
    int param_blob_length_ = 6;
    std::vector<float> mean_ = {0, 0, 0};
    std::vector<PadResizeParam> pad_resize_params_;
    base::Image8U padding_image_container_;
    int gpu_id_ = 0;
    Select select_;

    std::shared_ptr<IGetBox> crop_;
    cudaStream_t stream_ = 0;

    DISALLOW_COPY_AND_ASSIGN(TrafficLightTLDetectorYolox);
};

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
