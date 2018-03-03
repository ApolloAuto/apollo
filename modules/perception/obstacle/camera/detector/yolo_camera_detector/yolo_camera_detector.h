/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_YOLO_CAMERA_DETECTOR_H_

#include <caffe/caffe.hpp>
#include "modules/lib/config_manager/config_manager.h"
#include "modules/obstacle/camera/common/cnn_adapter.h"
#include "modules/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/proto/yolo.pb.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/region_output.h"
#include "modules/obstacle/camera/dummy/dummy_algorithms.h"
#include "modules/obstacle/camera/interface/base_camera_detector.h"

namespace apollo {
namespace perception {
namespace obstacle {

class YoloCameraDetector : public BaseCameraDetector {
 public:
  YoloCameraDetector() : BaseCameraDetector() {}

  virtual ~YoloCameraDetector() {}

  virtual bool init(const CameraDetectorInitOptions &options =
                        CameraDetectorInitOptions()) override;

  virtual bool detect(const cv::Mat &frame,
                      const CameraDetectorOptions &options,
                      std::vector<VisualObjectPtr> *objects) override;

  virtual bool multitask(const cv::Mat &frame,
                         const CameraDetectorOptions &options,
                         std::vector<VisualObjectPtr> *objects, cv::Mat *mask);

  virtual bool extract(std::vector<VisualObjectPtr> *objects) {
    for (auto &extractor : extractors_) {
      extractor->extract(objects);
    }
    return true;
  }

  virtual std::string name() const override;

 protected:
  bool get_objects_gpu(std::vector<VisualObjectPtr> *objects);

  bool init_cnn(const std::string &yolo_root);

  void load_intrinsic(const CameraDetectorInitOptions &options);

  void load_nms_params();

  void init_anchor(const std::string &yolo_root);

 private:
  std::shared_ptr<CNNAdapter> cnnadapter_;

  boost::shared_ptr<anakin::Tensor<float>> res_cls_tensor_ = nullptr;
  boost::shared_ptr<anakin::Tensor<float>> res_box_tensor_ = nullptr;
  std::shared_ptr<SyncedMemory> image_data_ = nullptr;
  std::shared_ptr<SyncedMemory> overlapped_ = nullptr;
  std::shared_ptr<SyncedMemory> idx_sm_ = nullptr;
  std::shared_ptr<SyncedMemory> anchor_ = nullptr;
  int height_ = 0;
  int width_ = 0;
  float _min_2d_height = 0;
  float _min_3d_height = 0;
  float top_k_ = 1000;
  int obj_size_ = 0;
  int output_height_ = 0;
  int output_width_ = 0;
  int lane_output_height_ = 0;
  int lane_output_width_ = 0;
  int num_anchors_ = 10;
  std::vector<boost::shared_ptr<BaseFeatureExtractor>> extractors_;

  std::vector<ObjectType> types_;
  int offset_y_ = 0;
  NMSParam nms_;
  float inter_cls_nms_thresh_ = 1;
  float cross_class_merge_threshold_ = 1;
  float confidence_threshold_ = 0.1;
  std::shared_ptr<BaseProjector> projector_;
  adu::perception::obstacle::yolo::YoloParam yolo_param_;
  int image_height_ = 0;
  int image_width_ = 0;
};

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_YOLO_CAMERA_DETECTOR_H_
