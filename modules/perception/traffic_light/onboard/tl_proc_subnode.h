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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TL_PROC_SUBNODE_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TL_PROC_SUBNODE_H_

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "gflags/gflags.h"

#include "modules/perception/onboard/common_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/traffic_light/interface/base_recognizer.h"
#include "modules/perception/traffic_light/interface/base_rectifier.h"
#include "modules/perception/traffic_light/interface/base_reviser.h"
#include "modules/perception/traffic_light/interface/green_interface.h"
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct ImageLights;
class TLPreprocessingData;
class TLProcData;

class TLProcSubnode : public Subnode {
 public:
  TLProcSubnode() = default;
  ~TLProcSubnode();
  apollo::common::Status ProcEvents() override;

 protected:
  bool InitInternal() override;
  bool ProcEvent(const Event &event);

 private:
  bool InitSharedData();
  bool InitRectifier();
  bool InitRecognizer();
  bool InitReviser();

  // get mean distance from car to stopline.
  double GetMeanDistance(const double ts, const Eigen::Matrix4d &car_location,
                         const LightPtrs &lights) const;

  bool VerifyImageLights(const ImageLights &image_lights,
                         CameraId *selection) const;

  // @brief compute image border size based on projection box and detection box
  bool ComputeImageBorder(const ImageLights &image_lights, int *image_border);

  // @brief compute offset between two rectangles
  void ComputeRectsOffset(const cv::Rect &rect1, const cv::Rect &rect2,
                          int *offset);
  bool PublishMessage(const std::shared_ptr<ImageLights> &image_lights);

 private:
  int image_border_ = 100;
  float valid_ts_interval_;
  TLPreprocessingData *preprocessing_data_ = nullptr;  // up-stream data
  std::unique_ptr<BaseRectifier> rectifier_ = nullptr;
  std::unique_ptr<BaseRecognizer> recognizer_ = nullptr;
  std::unique_ptr<BaseReviser> reviser_ = nullptr;
  Mutex mutex_;
  DISALLOW_COPY_AND_ASSIGN(TLProcSubnode);
};

REGISTER_SUBNODE(TLProcSubnode);
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_TL_PROC_SUBNODE_H_
