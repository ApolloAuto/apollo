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

#include "modules/prediction/predictor/vehicle/free_move_predictor.h"

#include <vector>
#include "Eigen/Dense"

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/proto/feature.pb.h"

namespace apollo {
namespace prediction {

using ::apollo::common::TrajectoryPoint;

void FreeMovePredictor::Predict(Obstacle* obstacle) {
  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  const Feature& feature = obstacle->latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle->id()
           << " is missing position or velocity";
    return;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  Eigen::Vector2d velocity(feature.velocity().x(), feature.velocity().y());
  if (FLAGS_enable_kf_tracking) {
    position(0) = feature.t_position().x();
    position(1) = feature.t_position().y();
    velocity(0) = feature.t_velocity().x();
    velocity(1) = feature.t_velocity().y();
  }

  std::vector<TrajectoryPoint> points(0);
  double total_time = FLAGS_prediction_duration;
  // TODO(kechxu):
  // draw_free_move_trajectory(position, velocity,
  // obstacle->kf_motion_tracker(),
  //                           total_time, points);
  Trajectory trajectory;
  GenerateTrajectory(points, &trajectory);
  int start_index = 0;
  prediction_obstacle_.set_predicted_period(total_time);
  prediction_obstacle_.add_trajectory()->CopyFrom(trajectory);
  SetEqualProbability(1.0, start_index);
}

}  // namespace prediction
}  // namespace apollo
