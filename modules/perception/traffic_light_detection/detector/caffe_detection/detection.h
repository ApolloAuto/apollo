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
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/traffic_light_detection/detector/caffe_detection/proto/model_param.pb.h"

#include "cyber/common/macros.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/image_8u.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/traffic_light_detection/algorithm/cropbox.h"
#include "modules/perception/traffic_light_detection/algorithm/select.h"
#include "modules/perception/traffic_light_detection/interface/base_traffic_light_detector.h"

namespace apollo {
namespace perception {
namespace trafficlight {

class TrafficLightDetection : public BaseTrafficLightDetector {
public:
    /**
     * @brief Construct a new traffic light detection object.
     *
     */
    TrafficLightDetection();
    /**
     * @brief Destroy the traffic light detection object.
     *
     */
    ~TrafficLightDetection() = default;
    /**
     * @brief Initialize traffic light detector parameters.
     *
     * @param options
     * @return true
     * @return false
     */
    bool Init(const TrafficLightDetectorInitOptions &options) override;

    /**
     * @brief Detect traffic light from image.
     *
     * @param frame
     * @return true
     * @return false
     */
    bool Detect(camera::TrafficLightFrame *frame) override;
    /**
     * @brief Dump output of inference results.
     *
     * @param crop_box_list
     * @param resize_scale_list_col
     * @param resize_scale_list_row
     * @param lights
     * @return true
     * @return false
     */
    bool SelectOutputBoxes(
            const std::vector<base::RectI> &crop_box_list,
            const std::vector<float> &resize_scale_list_col,
            const std::vector<float> &resize_scale_list_row,
            std::vector<base::TrafficLightPtr> *lights);
    /**
     * @brief Filter overlapping boxes using NMS.
     *
     * @param lights
     * @param iou_thresh
     */
    void ApplyNMS(std::vector<base::TrafficLightPtr> *lights, double iou_thresh = 0.6);
    /**
     * @brief Model inference process.
     *
     * @param lights
     * @param data_provider
     * @return true
     * @return false
     */
    bool Inference(std::vector<base::TrafficLightPtr> *lights, camera::DataProvider *data_provider);
    /**
     * @brief Get the detected boxes object.
     *
     * @return const std::vector<base::TrafficLightPtr>&
     */
    const std::vector<base::TrafficLightPtr> &getDetectedBoxes() {
        return detected_bboxes_;
    }

private:
    trafficlight::ModelParam detection_param_;
    std::string detection_root_dir;

    camera::DataProvider::ImageOptions data_provider_image_option_;
    std::shared_ptr<inference::Inference> rt_net_ = nullptr;
    std::shared_ptr<base::Image8U> image_ = nullptr;
    std::shared_ptr<base::Blob<float>> param_blob_;
    std::shared_ptr<base::Blob<float>> mean_buffer_;
    std::shared_ptr<IGetBox> crop_;
    std::vector<base::TrafficLightPtr> detected_bboxes_;
    std::vector<base::TrafficLightPtr> selected_bboxes_;
    std::vector<std::string> net_inputs_;
    std::vector<std::string> net_outputs_;
    Select select_;
    int max_batch_size_;
    int param_blob_length_;
    float mean_[3];
    std::vector<base::RectI> crop_box_list_;
    std::vector<float> resize_scale_list_;
    int gpu_id_;

    DISALLOW_COPY_AND_ASSIGN(TrafficLightDetection);
};  // class TrafficLightDetection

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
