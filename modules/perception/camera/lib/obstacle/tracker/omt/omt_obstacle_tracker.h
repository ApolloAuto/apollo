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

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/lib/interface/base_obstacle_tracker.h"
#include "modules/perception/camera/lib/obstacle/tracker/common/similar.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/obstacle_reference.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/omt.pb.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/target.h"

namespace apollo {
namespace perception {
namespace camera {
struct Hypothesis {
  int target;
  int object;
  float score;

  Hypothesis() {
    this->target = -1;
    this->object = -1;
    this->score = -1;
  }

  Hypothesis(int tar, int obj, float score) {
    this->target = tar;
    this->object = obj;
    this->score = score;
  }

  bool operator<(const Hypothesis &b) const { return score < b.score; }

  bool operator>(const Hypothesis &b) const { return score > b.score; }
};

class OMTObstacleTracker : public BaseObstacleTracker {
 public:
  //  OMTObstacleTracker() : similar_(nullptr), track_id_(0),
  //                         frame_num_(0), gpu_id_(0),
  //                         width_(0.0f), height_(0.0f),
  //                         BaseObstacleTracker() {
  //  }
  OMTObstacleTracker() : BaseObstacleTracker() {}

  ~OMTObstacleTracker() override = default;

  bool Init(const ObstacleTrackerInitOptions &options) override;
  // @brief: predict candidate obstales in the new image.
  // @param [in]: options
  // @param [in/out]: frame
  // candidate obstacle 2D boxes should be filled, required.
  bool Predict(const ObstacleTrackerOptions &options,
               CameraFrame *frame) override;

  // @brief: associate obstales by 2D information.
  // @param [in]: options
  // @param [in/out]: frame
  // associated obstacles with tracking id should be filled, required,
  // smoothed 2D&3D information can be filled, optional.
  bool Associate2D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame) override;

  // @brief: associate obstales by 3D information.
  // @param [in]: options
  // @param [in/out]: frame
  // associated obstacles with tracking id should be filled, required,
  // smoothed 3D information can be filled, optional.
  bool Associate3D(const ObstacleTrackerOptions &options,
                   CameraFrame *frame) override;

  // @brief: track detected obstacles.
  // @param [in]: options
  // @param [in/out]: frame
  // associated obstacles with tracking id should be filled, required,
  // motion information of obstacles should be filled, required.
  bool Track(const ObstacleTrackerOptions &options,
             CameraFrame *frame) override;

  std::string Name() const override;

 private:
  float ScoreAppearance(const Target &target, TrackObjectPtr object);

  float ScoreMotion(const Target &target, TrackObjectPtr object);
  float ScoreShape(const Target &target, TrackObjectPtr object);
  float ScoreOverlap(const Target &target, TrackObjectPtr track_obj);
  void ClearTargets();
  bool CombineDuplicateTargets();
  void GenerateHypothesis(const TrackObjectPtrs &objects);
  int CreateNewTarget(const TrackObjectPtrs &objects);

 private:
  omt::OmtParam omt_param_;
  FrameList frame_list_;
  SimilarMap similar_map_;
  std::shared_ptr<BaseSimilar> similar_ = nullptr;
  std::vector<Target> targets_;
  std::vector<bool> used_;
  ObstacleReference reference_;
  std::vector<std::vector<float> > kTypeAssociatedCost_;
  int track_id_ = 0;
  int frame_num_ = 0;
  int gpu_id_ = 0;
  float width_ = 0.0f;
  float height_ = 0.0f;

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
