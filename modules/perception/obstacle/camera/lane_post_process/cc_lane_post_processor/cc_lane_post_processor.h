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

// @brief: CC lane post-processor header file

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/interface/base_lane_post_processor.h"
#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/lane_frame.h"

namespace apollo {
namespace perception {

struct CCLanePostProcessorOptions {
  SpaceType space_type;
  ScalarType lane_map_conf_thresh;
  ScalarType cc_split_siz;
  int cc_split_len;
  LaneFrameOptions frame;

  CCLanePostProcessorOptions()
      : space_type(SpaceType::VEHICLE),
        lane_map_conf_thresh(0.5),
        cc_split_siz(100.0),
        cc_split_len(50) {}
};

class CCLanePostProcessor : public BaseCameraLanePostProcessor {
 public:
  CCLanePostProcessor() : BaseCameraLanePostProcessor() {
    max_distance_to_see_ = 200.0;
    vis_ = false;
    is_init_ = false;
  }

  ~CCLanePostProcessor() {}

  bool Init() override;

  bool Process(const cv::Mat &lane_map,
               const CameraLanePostProcessOptions &options,
               LaneObjectsPtr lane_instances) override;

  void set_max_distance_to_see(ScalarType max_distance_to_see) {
    max_distance_to_see_ = max_distance_to_see;
  }
  void set_vis(bool vis) {
    vis_ = vis;
  }

  std::string name() const {
    return "CCLanePostProcessor";
  }

  CCLanePostProcessorOptions options() const {
    return options_;
  }

  cv::Rect roi() const {
    return roi_;
  }

  const std::shared_ptr<LaneFrame> &cur_frame() {
    return cur_frame_;
  }

  std::shared_ptr<std::vector<LaneInstance>> &cur_lane_instances() {
    return cur_lane_instances_;
  }

 protected:
  // @brief: add instance into a lane object
  bool AddInstanceIntoLaneObject(const LaneInstance &instance,
                                 LaneObject *lane_object);

  // @brief: add instance into a lane object (for "image" mode)
  bool AddInstanceIntoLaneObjectImage(const LaneInstance &instance,
                                      LaneObject *lane_object);

  // @brief: generate lane instances from lane map (using lane_frame)
  bool GenerateLaneInstances(const cv::Mat &lane_map);

  bool CompensateLaneObjects(LaneObjectsPtr lane_objects);

  bool EnrichLaneInfo(LaneObjectsPtr lane_objects);

 private:
  CCLanePostProcessorOptions options_;

  double time_stamp_;
  int frame_id_;
  std::shared_ptr<ConnectedComponentGenerator> cc_generator_;
  std::shared_ptr<LaneFrame> cur_frame_;
  LaneInstancesPtr cur_lane_instances_;

  ScalarType max_distance_to_see_;
  int image_width_;
  int image_height_;
  cv::Rect roi_;

  bool is_x_longitude_;

  std::shared_ptr<Projector<ScalarType>> projector_;

  bool is_init_;
  bool vis_;

  DISALLOW_COPY_AND_ASSIGN(CCLanePostProcessor);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_H_
