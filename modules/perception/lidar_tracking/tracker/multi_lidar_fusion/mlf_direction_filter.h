/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include "modules/perception/common/base/object.h"
#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_base_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class MlfDirectionFilter : public MlfBaseFilter {
 public:
  MlfDirectionFilter() = default;
  virtual ~MlfDirectionFilter() = default;

  bool Init(const MlfFilterInitOptions& options = MlfFilterInitOptions())
      override;

  std::string Name() const override {
    return "MlfDirectionFilter";
  }
  inline double Degree2Angle(double degree) {
    return degree * 180.0 / M_PI;
  }
  inline double Degree2Radian(double angle) {
    return angle * M_PI / 180.0;
  }

  void UpdateWithObject(const MlfFilterOptions& options,
                        const MlfTrackDataConstPtr& track_data,
                        TrackedObjectPtr new_object) override;
  void UpdateWithoutObject(const MlfFilterOptions& options, double timestamp,
                           MlfTrackDataPtr track_data) override;

 protected:
  void InitializeTrackState(TrackedObjectPtr new_object);

  void KalmanFilterUpdateDirection(const MlfTrackDataConstPtr& track_data,
                                     const TrackedObjectConstPtr& latest_object,
                                     TrackedObjectPtr new_object);

  void ComputeVarianceAndAngular(
    const MlfTrackDataConstPtr& track_data, TrackedObjectPtr new_object,
    double* direction_measurement_variance,
    double* angular_measurement_variance,
    double* angular);

  void ComputeMeasurementConfidence(
    const TrackedObjectPtr& new_object, double* confidence);

  void ComputeConvergenceConfidence(
    const MlfTrackDataConstPtr& track_data, TrackedObjectPtr new_object);

  double VecDiffAngle(const Eigen::Vector3d& vec0, const Eigen::Vector3d& vec1);

  double Vec2Angle(const Eigen::Vector3d& vec);

  double LegalizeAngle(double angle);

  void GetDirectionState(TrackedObjectPtr& new_object);

 private:
  // kalman filter  parameters
  double init_direction_variance_ = 100.0;
  double predict_noise_per_sqrsec_ = 100.0;
  double variance_bound_ = 25.0;
  double max_angular_thresh_ = 50.0;
  double converged_confidence_minimum_ = 0.6;
  double angular_scale_ = 0.8;
  double angular_confidence_thresh_ = 0.7;
  size_t angular_window_size_ = 5;
};  // class MlfDirectionFilter

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
