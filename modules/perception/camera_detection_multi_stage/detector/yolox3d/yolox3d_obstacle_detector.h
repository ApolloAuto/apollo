/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "modules/perception/camera_detection_multi_stage/detector/yolox3d/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class Yolox3DObstacleDetector : public BaseObstacleDetector {
public:
    Yolox3DObstacleDetector() : BaseObstacleDetector() {}
    virtual ~Yolox3DObstacleDetector() {
        if (stream_ != nullptr) {
            cudaStreamDestroy(stream_);
        }
    }

    /**
    * @brief Necessary, Init yolox3d model params normal,
             but now use pipline instead
    * @param options obstacle detection init options
    * @return init status, yolox3d detector stage status
    */
    bool Init(const ObstacleDetectorInitOptions &options = ObstacleDetectorInitOptions()) override;

    bool Detect(onboard::CameraFrame *frame) override;

    /**
     * @brief return detector name
     * @param None
     * @return now detector type
     */
    std::string Name() const override {
        return "Yolox3DObstacleDetector";
    }

protected:
    /**
    * @brief Preprocess of image before inference,
             resize input data blob and fill image data to blob
    * @param image image read from camera frame of 6mm camera
    * @param input_blob image input blob address pointer
    * @return preprocess status
    */
    bool Preprocess(const base::Image8U *image, base::BlobPtr<float> input_blob);

    /**
     * @brief Resize model input picture size according to the config
     *        file
     * @param model_param yolox3d proto param read from yolox3d.pt
     * @return None
     */
    void LoadInputShape(const yolox3d::ModelParam &model_param);

    /**
    * @brief Load yolo libtorch model params from model file
    * @param yolox3d_param yolox3d proto param read from yolox3d.pt,
              include ModelParam、NetworkParam and NMSParam
    * @return None
    */
    void LoadParam(const yolox3d::ModelParam &model_param);
    /**
     * @brief Load yolo3D libtorch model params from model file
     *
     * @param image
     * @param detect_objs
     */
    void Yolo3DInference(const base::Image8U *image, std::vector<base::ObjectPtr> &detect_objs);
    /**
     * @brief Init model inference
     *
     * @param model_info
     * @param model_path
     * @return true
     * @return false
     */
    bool Init3DNetwork(const common::ModelInfo &model_info, const std::string &model_path);

private:
    ObstacleDetectorInitOptions options_;
    yolox3d::ModelParam model_param_;
    yolox3d::NMSParam nms_;

    std::shared_ptr<inference::Inference> net_3D_;
    FILE *file_;

    int gpu_id_ = 0;
    int max_batch_size_ = 32;
    cudaStream_t stream_ = nullptr;

    // yolo input image size
    int width_ = 0;
    int height_ = 0;
    // image size of raw image
    int image_width_ = 0;
    int image_height_ = 0;
    float confidence_threshold_ = 0.f;

    float border_ratio_ = 0.f;
    cv::Mat img_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
