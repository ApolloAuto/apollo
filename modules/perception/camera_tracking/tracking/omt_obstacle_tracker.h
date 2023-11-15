/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <boost/circular_buffer.hpp>

#include "modules/perception/camera_tracking/proto/omt.pb.h"

#include "modules/perception/camera_tracking/base/obstacle_reference.h"
#include "modules/perception/camera_tracking/base/target.h"
#include "modules/perception/camera_tracking/common/camera_tracking_frame.h"
#include "modules/perception/camera_tracking/common/similar.h"
#include "modules/perception/camera_tracking/interface/base_feature_extractor.h"
#include "modules/perception/camera_tracking/interface/base_obstacle_tracker.h"
#include "modules/perception/common/camera/common/object_template_manager.h"

namespace apollo {
namespace perception {
namespace camera {

struct alignas(16) Hypothesis {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int target;
  int object;
  float score;

  Hypothesis() : target(-1), object(-1), score(-1) {}

  Hypothesis(int tar, int obj, float score)
      : target(tar), object(obj), score(score) {}

  bool operator<(const Hypothesis &b) const { return score < b.score; }

  bool operator>(const Hypothesis &b) const { return score > b.score; }
};

class OMTObstacleTracker : public BaseObstacleTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OMTObstacleTracker() = default;
  ~OMTObstacleTracker() = default;

  /**
   * @brief initialize omt obstacle tracker with options
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const ObstacleTrackerInitOptions &options) override;

  /**
   * @brief main function for tracking
   *
   * @param camera_frame
   * @return true
   * @return false
   */
  bool Process(std::shared_ptr<CameraTrackingFrame> camera_frame);

  /**
   * @brief extract features from the new frame
   *
   * @param frame
   * @return true
   * @return false
   */
  bool FeatureExtract(CameraTrackingFrame *frame) override;

  /**
   * @brief predict candidate obstales in the new frame
   * candidate obstacle 2D boxes should be filled, required
   *
   * @param frame
   * @return true
   * @return false
   */
  bool Predict(CameraTrackingFrame *frame) override;

  /**
   * @brief calculate similarity of two objects
   *
   * @param frame
   * @return true
   * @return false
   */
  // bool CalSimilarity(CameraTrackingFrame *frame) override;

  /**
   * @brief associate obstales by 2D information
   * associated obstacles with tracking id should be filled, required,
   * smoothed 2D&3D information can be filled, optional.
   * @param frame
   * @return true
   * @return false
   */
  bool Associate2D(std::shared_ptr<CameraTrackingFrame> frame) override;

  /**
   * @brief associate obstales by 3D information
   * associated obstacles with tracking id should be filled, required,
   * smoothed 3D information can be filled, optional.
   * @param frame
   * @return true
   * @return false
   */
  bool Associate3D(std::shared_ptr<CameraTrackingFrame> frame) override;

 private:
  /**
   * @brief calculate the feature similarity between Target and object
   *
   * @param target
   * @param object
   * @return float
   */
  float ScoreAppearance(const Target &target, TrackObjectPtr object);

  /**
   * @brief calculate the motion similarity between Target and object
   *
   * @param target
   * @param object
   * @return float
   */
  float ScoreMotion(const Target &target, TrackObjectPtr object);
  /**
   * @brief calculate the shape similarity between Target and object
   *
   * @param target
   * @param object
   * @return float
   */
  float ScoreShape(const Target &target, TrackObjectPtr object);
  /**
   * @brief calculate the overlap similarity (iou) between Target and object
   *
   * @param target
   * @param track_obj
   * @return float
   */
  float ScoreOverlap(const Target &target, TrackObjectPtr track_obj);
  /**
   * @brief clear non Target in targets_
   *
   */
  void ClearTargets();
  /**
   * @brief combine duplicate Targets
   *
   * @return true
   * @return false
   */
  bool CombineDuplicateTargets();
  /**
   * @brief generate hypothesis
   *
   * @param objects
   */
  void GenerateHypothesis(const TrackObjectPtrs &objects);
  /**
   * @brief create new Target
   *
   * @param objects
   * @return int
   */
  int CreateNewTarget(const TrackObjectPtrs &objects);

 private:
  std::shared_ptr<BaseFeatureExtractor> feature_extractor_;
  OmtParam omt_param_;
  boost::circular_buffer<std::shared_ptr<CameraTrackingFrame>> frame_buffer_;
  SimilarMap similar_map_;
  std::shared_ptr<BaseSimilar> similar_ = nullptr;
  // Target is one tracked object in a sequence
  // targets_ is all the tracked objects in a sequence
  apollo::common::EigenVector<Target> targets_;
  std::vector<bool> used_;
  ObstacleReference reference_;
  std::vector<std::vector<float>> kTypeAssociatedCost_;
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
