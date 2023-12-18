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
#include <vector>

#include <boost/circular_buffer.hpp>

#include "modules/perception/camera_tracking/proto/omt.pb.h"

#include "modules/perception/camera_tracking/base/track_object.h"
#include "modules/perception/camera_tracking/common/camera_tracking_frame.h"
#include "modules/perception/camera_tracking/common/kalman_filter.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/camera/common/object_template_manager.h"

namespace apollo {
namespace perception {
namespace camera {

struct alignas(16) Target {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Target(const TargetParam &param);
  /**
   * @brief init Target
   *
   * @param param
   */
  void Init(const TargetParam &param);
  /**
   * @brief add object to tracked_objects
   *
   * @param object
   */
  void Add(TrackObjectPtr object);

  /**
   * @brief remove objects older than frame_id
   *
   * @param frame_id
   */
  void RemoveOld(int frame_id);

  /**
   * @brief clear tracked_objects
   */
  void Clear();

  /**
   * @brief using kalman filter to predict the tracked_objects
   *
   * @param frame
   */
  void Predict(CameraTrackingFrame *frame);

  /**
   * @brief using kalman filter to correct the tracked_objects
   * todo(zero): update world in bev
   * @param frame
   */
  void Update(CameraTrackingFrame *frame);

  /**
   * @brief update 2d
   *
   * @param frame
   */
  void Update2D(CameraTrackingFrame *frame);

  /**
   * @brief update 3d
   *
   * @param frame
   */
  void Update3D(CameraTrackingFrame *frame);

  /**
   * @brief update type
   *
   * @param frame
   */
  void UpdateType(CameraTrackingFrame *frame);

  /**
   * @brief return the size of tracked_objects
   *
   * @return int
   */
  int Size() const;

  /**
   * @brief Get the object accoding to index
   *
   * @param index
   * @return TrackObjectPtr
   */
  TrackObjectPtr get_object(int index) const;
  TrackObjectPtr operator[](int index) const;

  /**
   * @brief return whether the target is tracked
   *
   * @return true
   * @return false
   */
  bool isTracked() const;

  /**
   * @brief return whether the target is lost
   *
   * @return true
   * @return false
   */
  bool isLost() const;

 public:
  int lost_age = 0;
  int id = 0;
  double start_ts = 0.0;
  FirstOrderRCLowPassFilter direction;
  TrackObjectPtr latest_object = nullptr;
  base::ObjectSubType type = base::ObjectSubType::MAX_OBJECT_TYPE;
  KalmanFilterConstVelocity world_center;
  MeanFilter world_center_for_unmovable;

  // constant position kalman state
  KalmanFilterConstState<2> world_center_const;

  // displacement theta
  MeanFilter displacement_theta;

  MaxNMeanFilter world_lwh;
  MeanFilter world_lwh_for_unmovable;
  MeanFilter world_velocity;
  std::vector<float> type_probs;
  TargetParam target_param_;

  FirstOrderRCLowPassFilter image_wh;
  KalmanFilterConstVelocity image_center;

  TrackObjectPtrs tracked_objects;

 private:
  static int global_track_id;
  // clapping unreasonable velocities by strategy
  void ClappingTrackVelocity(const base::ObjectPtr &obj);
  bool CheckStatic();

  boost::circular_buffer<std::shared_ptr<base::Object>> history_world_states_;

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
