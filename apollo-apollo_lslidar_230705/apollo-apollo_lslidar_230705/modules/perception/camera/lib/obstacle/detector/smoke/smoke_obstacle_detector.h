/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include <utility>
#include <vector>

#include "modules/perception/camera/lib/obstacle/detector/smoke/proto/smoke.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/base/box.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/detector/smoke/region_output.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/region_output.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/utils/resize.h"
#include "modules/perception/inference/utils/util.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class SmokeObstacleDetector : public BaseObstacleDetector {
 public:
  SmokeObstacleDetector() : BaseObstacleDetector() {}
  virtual ~SmokeObstacleDetector() {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override;

  bool Init(const StageConfig& stage_config) override;

  bool Detect(const ObstacleDetectorOptions &options,
              CameraFrame *frame) override;

  bool Detect(const std::vector<float> &k_inv,
              const std::vector<float> &image_data_array,
              const float *detect_result);

  bool Process(DataFrame *data_frame) override;

  bool Process(const std::vector<float> &k_inv,
               const std::vector<float> &image_data_array,
               const float *detect_result);

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 protected:
  bool Preprocessor(const base::Image8U* image,
                    std::shared_ptr<base::Blob<float>> input_blob);
  void LoadInputShape(const smoke::ModelParam &model_param);
  void LoadParam(const smoke::SmokeParam &smoke_param);
  bool InitNet(const smoke::SmokeParam &smoke_param,
               const std::string &model_root);
  void InitSmokeBlob(const smoke::NetworkParam &net_param);
  bool InitFeatureExtractor(const std::string &root_dir);

 private:
  std::shared_ptr<BaseFeatureExtractor> feature_extractor_;
  smoke::SmokeParam smoke_param_;
  std::shared_ptr<base::BaseCameraModel> base_camera_model_ = nullptr;
  std::shared_ptr<inference::Inference> inference_;
  std::vector<base::ObjectSubType> types_;
  std::vector<float> expands_;
  std::vector<float> anchors_;
  std::vector<std::string> camera_names_;

  SmokeNMSParam nms_;
  cudaStream_t stream_ = nullptr;
  int height_ = 0;
  int width_ = 0;
  int offset_y_ = 0;
  int gpu_id_ = 0;
  // int obj_k_ = kMaxObjSize;
  int obj_k_ = 1000;

  int ori_cycle_ = 1;
  float confidence_threshold_ = 0.f;
  float light_vis_conf_threshold_ = 0.f;
  float light_swt_conf_threshold_ = 0.f;
  SmokeMinDims min_dims_;
  SmokeBlobs smoke_blobs_;

  std::shared_ptr<base::Image8U> image_ = nullptr;
  std::shared_ptr<base::Blob<bool>> overlapped_ = nullptr;
  std::shared_ptr<base::Blob<int>> idx_sm_ = nullptr;

  bool with_box3d_ = false;
  bool with_frbox_ = false;
  bool with_lights_ = false;
  bool with_ratios_ = false;
  bool with_area_id_ = false;
  float border_ratio_ = 0.f;

  SmokeObstacleDetectionConfig smoke_obstacle_detection_config_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
