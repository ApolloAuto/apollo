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

#include "modules/perception/camera/proto/yolo.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/base/box.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/region_output.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/utils/resize.h"
#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace camera {

class YoloObstacleDetector : public BaseObstacleDetector {
 public:
  YoloObstacleDetector() : BaseObstacleDetector() {}
  virtual ~YoloObstacleDetector() {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override;

  bool Detect(const ObstacleDetectorOptions &options,
              CameraFrame *frame) override;
  std::string Name() const override { return "YoloObstacleDetector"; }

 protected:
  void LoadInputShape(const yolo::ModelParam &model_param);
  void LoadParam(const yolo::YoloParam &yolo_param);
  bool InitNet(const yolo::YoloParam &yolo_param,
               const std::string &model_root);
  void InitYoloBlob(const yolo::NetworkParam &net_param);
  bool InitFeatureExtractor(const std::string &root_dir);

 private:
  std::shared_ptr<BaseFeatureExtractor> feature_extractor_;
  yolo::YoloParam yolo_param_;
  std::shared_ptr<base::BaseCameraModel> base_camera_model_ = nullptr;
  std::shared_ptr<inference::Inference> inference_;
  std::vector<base::ObjectSubType> types_;
  std::vector<float> expands_;
  std::vector<float> anchors_;

  NMSParam nms_;
  cudaStream_t stream_ = nullptr;
  int height_ = 0;
  int width_ = 0;
  int offset_y_ = 0;
  int gpu_id_ = 0;
  int obj_k_ = kMaxObjSize;

  int ori_cycle_ = 1;
  float confidence_threshold_ = 0.f;
  float light_vis_conf_threshold_ = 0.f;
  float light_swt_conf_threshold_ = 0.f;
  MinDims min_dims_;
  YoloBlobs yolo_blobs_;

  std::shared_ptr<base::Image8U> image_ = nullptr;
  std::shared_ptr<base::Blob<bool>> overlapped_ = nullptr;
  std::shared_ptr<base::Blob<int>> idx_sm_ = nullptr;

  bool with_box3d_ = false;
  bool with_frbox_ = false;
  bool with_lights_ = false;
  bool with_ratios_ = false;
  bool with_area_id_ = false;
  float border_ratio_ = 0.f;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
