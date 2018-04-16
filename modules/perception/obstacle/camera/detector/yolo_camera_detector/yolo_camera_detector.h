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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_YOLO_CAMERA_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_YOLO_CAMERA_DETECTOR_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "caffe/caffe.hpp"

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/proto/yolo.pb.h"
#include "modules/perception/proto/yolo_camera_detector_config.pb.h"

#include "modules/perception/cuda_util/network.h"
#include "modules/perception/cuda_util/region_output.h"
#include "modules/perception/cuda_util/util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/common/cnn_adapter.h"
#include "modules/perception/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/perception/obstacle/camera/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"

namespace apollo {
namespace perception {

class YoloCameraDetector : public BaseCameraDetector {
 public:
  YoloCameraDetector() : BaseCameraDetector() {}

  virtual ~YoloCameraDetector() {}

  bool Init(const CameraDetectorInitOptions &options =
                CameraDetectorInitOptions()) override;

  bool Detect(const cv::Mat &frame, const CameraDetectorOptions &options,
              std::vector<std::shared_ptr<VisualObject>> *objects) override;

  bool Multitask(const cv::Mat &frame, const CameraDetectorOptions &options,
                 std::vector<std::shared_ptr<VisualObject>> *objects,
                 cv::Mat *mask);

  bool Extract(std::vector<std::shared_ptr<VisualObject>> *objects) {
    for (auto &extractor : extractors_) {
      extractor->extract(objects);
    }
    return true;
  }

  std::string Name() const override;

 protected:
  bool init_cnn(const std::string &yolo_root);

  void load_intrinsic(const CameraDetectorInitOptions &options);

  void load_nms_params();

  void init_anchor(const std::string &yolo_root);

  bool get_objects_cpu(std::vector<std::shared_ptr<VisualObject>> *objects);
  bool get_objects_gpu(std::vector<std::shared_ptr<VisualObject>> *objects);

  void get_object_helper(int idx, const float *loc_data, const float *obj_data,
                         const float *cls_data, const float *ori_data,
                         const float *dim_data, const float *lof_data,
                         const float *lor_data, int width, int height,
                         int num_classes, bool with_ori, bool with_dim,
                         bool with_lof, bool with_lor);

 private:
  std::shared_ptr<CNNAdapter> cnnadapter_;

  std::shared_ptr<caffe::SyncedMemory> res_cls_tensor_ = nullptr;
  std::shared_ptr<caffe::SyncedMemory> res_box_tensor_ = nullptr;

  std::shared_ptr<caffe::SyncedMemory> image_data_ = nullptr;
  std::shared_ptr<caffe::SyncedMemory> overlapped_ = nullptr;
  std::shared_ptr<caffe::SyncedMemory> idx_sm_ = nullptr;
  std::shared_ptr<caffe::SyncedMemory> anchor_ = nullptr;
  int height_ = 0;
  int width_ = 0;
  float min_2d_height_ = 0.0f;
  float min_3d_height_ = 0.0f;
  int top_k_ = 1000;
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
  float inter_cls_nms_thresh_ = 1.0f;
  float cross_class_merge_threshold_ = 1.0f;
  float confidence_threshold_ = 0.1f;
  std::shared_ptr<BaseProjector> projector_;
  obstacle::yolo::YoloParam yolo_param_;
  int image_height_ = 0;
  int image_width_ = 0;

  yolo_camera_detector_config::ModelConfigs config_;
};

REGISTER_CAMERA_DETECTOR(YoloCameraDetector);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_YOLO_CAMERA_DETECTOR_H_
