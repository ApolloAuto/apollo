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

#include <boost/circular_buffer.hpp>
#include <vector>

#include "modules/perception/base/object.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/lib/obstacle/tracker/common/kalman_filter.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/omt.pb.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/track_object.h"

namespace apollo {
namespace perception {
namespace camera {

struct Target {
 public:
  explicit Target(const omt::TargetParam &param);
  void Init(const omt::TargetParam &param);
  void Add(TrackObjectPtr object);

  void RemoveOld(int frame_id);

  void Clear();

  void Predict(CameraFrame *frame);

  void Update2D(CameraFrame *frame);

  void Update3D(CameraFrame *frame);

  void UpdateType(CameraFrame *frame);

  int Size() const;
  TrackObjectPtr get_object(int index) const;
  TrackObjectPtr operator[](int index) const;

  bool isTracked() const;
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
  omt::TargetParam target_param_;

  FirstOrderRCLowPassFilter image_wh;
  KalmanFilterConstVelocity image_center;

  TrackObjectPtrs tracked_objects;

 private:
  static int global_track_id;
  // clapping unreasonable velocities by strategy
  void ClappingTrackVelocity(const base::ObjectPtr &obj);
  bool CheckStatic();

  boost::circular_buffer<base::Object> history_world_states_;

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
