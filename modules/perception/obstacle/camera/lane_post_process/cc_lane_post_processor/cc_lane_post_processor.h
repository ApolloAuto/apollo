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

#include <boost/circular_buffer.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/perception/proto/lane_post_process_config.pb.h"

#include "modules/common/log.h"
#include "modules/perception/cuda_util/connected_component_gpu.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/object_supplement.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/interface/base_lane_post_processor.h"
#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/lane_frame.h"

namespace apollo {
namespace perception {

#define CUDA_CC false
#define USE_HISTORY_TO_EXTEND_LANE false
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
  CCLanePostProcessor() : BaseCameraLanePostProcessor() {}

  ~CCLanePostProcessor() {}

  bool Init() override;

  bool Process(const cv::Mat &lane_map,
               const CameraLanePostProcessOptions &options,
               LaneObjectsPtr *lane_instances) override;

  void set_max_distance_to_see(ScalarType max_distance_to_see) {
    max_distance_to_see_ = max_distance_to_see;
  }
  void set_vis(bool vis) { vis_ = vis; }

  std::string name() const { return "CCLanePostProcessor"; }

  CCLanePostProcessorOptions options() const { return options_; }

  cv::Rect roi() const { return roi_; }

  const std::shared_ptr<LaneFrame> &cur_frame() { return cur_frame_; }

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

  void InitLaneHistory();

  void FilterWithLaneHistory(LaneObjectsPtr lane_objects);

  bool CorrectWithLaneHistory(int l, LaneObjectsPtr lane_objects,
                              std::vector<bool> *is_valid);
  bool FindLane(const LaneObjects &lane_objects, int spatial_label, int *index);

  void ExtendLaneWithHistory(const LaneObject &history, LaneObject *lane);

 private:
  CCLanePostProcessorOptions options_;

  std::shared_ptr<NonMask> non_mask_;

  double time_stamp_ = 0.0;
  int frame_id_ = -1;
#if CUDA_CC
  std::shared_ptr<ConnectedComponentGeneratorGPU> cc_generator_;
#else
  std::shared_ptr<ConnectedComponentGenerator> cc_generator_;
#endif
  std::shared_ptr<LaneFrame> cur_frame_;
  LaneInstancesPtr cur_lane_instances_;

  ScalarType max_distance_to_see_ = 500.0;
  int image_width_ = 1080;
  int image_height_ = 1920;
  cv::Rect roi_;

  double scale_;
  int start_y_pos_;
  bool is_x_longitude_ = true;

  std::shared_ptr<Projector<ScalarType>> projector_;

  bool is_init_ = false;
  bool vis_ = false;

  lane_post_process_config::ModelConfigs config_;

  bool use_history_ = false;
  boost::circular_buffer<LaneObjects> lane_history_;
  MotionBufferPtr motion_buffer_ = nullptr;
  const std::vector<SpatialLabelType> interested_labels_ = {
      SpatialLabelType::L_0, SpatialLabelType::R_0};
  LaneObjectsPtr generated_lanes_ = nullptr;
  DISALLOW_COPY_AND_ASSIGN(CCLanePostProcessor);
};

// Register plugin.
REGISTER_CAMERA_LANE_POST_PROCESSOR(CCLanePostProcessor);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_H_
