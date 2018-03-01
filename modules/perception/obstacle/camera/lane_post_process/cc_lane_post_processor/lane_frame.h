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

// @brief: lane detection on a single image frame

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_LANE_FRAME_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_LANE_FRAME_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/util.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/connected_component.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/projector.h"
#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/group.h"

namespace apollo {
namespace perception {

#define MAX_GRAPH_NUM 20

// #ifndef _DUMP_RESULT
// #define _DUMP_RESULT true
// #endif

struct LaneFrameOptions {
  // used for initialization
  SpaceType space_type;  // space type
  cv::Rect image_roi;
  bool use_cc;
  int min_cc_pixel_num;  // minimum number of pixels of CC
  int min_cc_size;       // minimum size of CC

  // used for greedy search association method
  int max_cc_marker_match_num;  // maximum number of markers used for matching
                                // for each CC

  // used for marker association
  ScalarType min_y_search_offset;  // minimum longitudinal offset used for
                                   // search upper markers
  AssociationParam assoc_param;
  GroupParam group_param;
  int orientation_estimation_skip_marker_num;

  // for determining lane object label
  ScalarType lane_interval_distance;  // predefined label interval distance

  // for fitting curve
  ScalarType min_instance_size_prefiltered;  // minimum size of lane instance to
                                             // be prefiltered
  ScalarType max_size_to_fit_straight_line;  // maximum size of instance to fit
                                             // a straight line

  LaneFrameOptions()
      : space_type(SpaceType::IMAGE),
        use_cc(true),
        min_cc_pixel_num(10),
        min_cc_size(5),
        max_cc_marker_match_num(1),
        min_y_search_offset(0.0),
        orientation_estimation_skip_marker_num(1),
        lane_interval_distance(4.0),
        min_instance_size_prefiltered(3.0),
        max_size_to_fit_straight_line(5.0) {}
};

class LaneFrame {
 public:
  LaneFrame() {}
  ~LaneFrame() {}

  bool Init(const std::vector<ConnectedComponentPtr>& input_cc,
            const LaneFrameOptions& options);

  bool Init(const std::vector<ConnectedComponentPtr>& input_cc,
            const std::shared_ptr<Projector>& projector,
            const LaneFrameOptions& options);

  void SetTransformer(const std::shared_ptr<Projector>& projector) {
    projector_ = projector;
    is_projector_init_ = true;
  }

  // void process();

  bool Process(std::vector<LaneInstance>* instances);

  int MarkerNum() const {
    return static_cast<int>(markers_.size());
  }

  bool IsValidMarker(int i) {
    return i >= 0 && i < MarkerNum();
  }

  const Marker* marker(int i) {
    return &(markers_.at(i));
  }

  int GraphNum() const {
    return static_cast<int>(graphs_.size());
  }

  const Graph* graph(int i) {
    return &(graphs_.at(i));
  }

  Bbox bbox(int i) const {
    return boxes_.at(i);
  }

  bool FitPolyCurve(const int& graph_id, const ScalarType& graph_siz,
                    PolyModel* poly_coef, ScalarType* lateral_distance) const;

 protected:
  ScalarType ComputeMarkerPairDistance(const Marker& ref, const Marker& tar);

  std::vector<int> ComputeMarkerEdges(
      std::vector<std::unordered_map<int, ScalarType>>* edges);

  bool GreedyGroupConnectAssociation();

  int AddGroupIntoGraph(const Group& group, Graph& graph,
                        std::unordered_set<int>& hash_marker_idx);

  int AddGroupIntoGraph(const Group& group, Graph& graph,
                        std::unordered_set<int>& hash_marker_idx,
                        int start_marker_ascend_id, int end_marker_descend_id);

  void ComputeBbox();

 private:
  LaneFrameOptions opts_;                                    // options
  std::vector<Marker> markers_;                              // markers
  int max_cc_num_;
  std::vector<int> cc_idx_;                                  // CC index for each marker
  std::unordered_map<int, std::vector<int>> cc_marker_lut_;  // marker indices for each CC

  std::shared_ptr<const Projector> projector_;
  bool is_projector_init_;

  std::vector<Graph> graphs_;                                // lane marker clusters
  std::vector<Bbox> boxes_;                                  // tight bounding boxes for lane cluster
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_LANE_FRAME_H_
