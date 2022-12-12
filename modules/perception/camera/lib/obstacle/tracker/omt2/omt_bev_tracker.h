/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/pipeline/proto/stage/omt.pb.h"

#include <string>
#include <vector>

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/lib/interface/base_obstacle_tracker.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/target.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/track_object.h"

namespace apollo {
namespace perception {
namespace camera {

struct HypothesisBev {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int target_idx;
  int object_idx;
  float score;
  HypothesisBev() : target_idx(-1), object_idx(-1), score(0.0f) {}
  HypothesisBev(int tar_idx, int obj_idx, float score)
      : target_idx(tar_idx), object_idx(obj_idx), score(score) {}

  bool operator<(const HypothesisBev& other) const {
    return score < other.score;
  }
  bool operator>(const HypothesisBev& other) const {
    return score > other.score;
  }
};

class OMTBEVTracker : public BaseObstacleTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool Init(const ObstacleTrackerInitOptions& options) override;

  // @brief: predict candidate obstales in the new image.
  // @param [in]: options
  // @param [in/out]: frame
  // candidate obstacle 2D boxes should be filled, required.
  bool Predict(const ObstacleTrackerOptions& options,
               CameraFrame* frame) override;

  bool Associate2D(const ObstacleTrackerOptions& options,
                   CameraFrame* frame) override;

  bool Associate3D(const ObstacleTrackerOptions& options,
                   CameraFrame* frame) override;

  bool Track(const ObstacleTrackerOptions& options,
             CameraFrame* frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  float ScoreAppearance(const Target& target, TrackObjectPtr object);
  float ScoreMotion(const Target& target, TrackObjectPtr object);
  float ScoreShape(const Target& target, TrackObjectPtr object);
  float ScoreOverlap(const Target& target, TrackObjectPtr object);

  void GenerateHypothesis(const TrackObjectPtrs& objects,
                          std::vector<HypothesisBev>* score_list);

  int CreateNewTarget(const TrackObjectPtrs& objects);

  void FindAbnormalTrack(TrackObjectPtrs* track_objects_ptr);

  bool CombineDuplicateTargets();
  void ClearTargets();

 private:
  int gpu_id_ = 0;
  FrameList frame_list_;

  omt::OmtParam omt_param_;
  std::vector<std::vector<float>> kTypeAssociatedCost_;

  apollo::common::EigenVector<Target> targets_;

 protected:
  ObjectTemplateManager* object_template_manager_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
