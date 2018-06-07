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

#ifndef MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_CONE_H_
#define MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_CONE_H_

#include <vector>
#include "Eigen/Core"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "modules/perception/obstacle/ultrasonic/common/extrin_ident.h"
#include "modules/perception/obstacle/ultrasonic/common/ultrasonic_kalman_filter.h"

namespace apollo {
namespace perception {

static const int kUltrasonicMaxLostCount = 5;

class BaseCone {
 public:
  BaseCone() {}
  virtual ~BaseCone() {}

 protected:
  int id_;
  float rho_;
  ExtrinIdent extrinsics_;
  bool meas_flag_;
  std::vector<Eigen::Vector3d> meas_arc_;
};

class RawCone : public BaseCone {
 public:
  RawCone() = default;

  RawCone(const int id, const std::vector<ExtrinIdent> &systmextrinsics_);

  RawCone(const int id, const ExtrinIdent &extrinsics);

  void cone_update(const float rho, const float time_diff);

  float rho() const { return rho_; }

  int Id() { return id_; }

  ExtrinIdent extrinsics() const { return extrinsics_; }

  bool meas_flag() const { return meas_flag_; }

  std::vector<Eigen::Vector3d> meas_arc() const { return meas_arc_; }
};

class FilteredCone : public BaseCone {
 public:
  FilteredCone() = default;

  FilteredCone(const int id, const ExtrinIdent& systm_extrinsic);

  void cone_update(const float rho, const double timestamp);

  float rho() const { return rho_; }

  int Id() { return id_; }

  ExtrinIdent extrinsics() const { return extrinsics_; }

  bool meas_flag() const { return meas_flag_; }

  std::vector<Eigen::Vector3d> meas_arc() const { return meas_arc_; }

 private:
  UltrasonicKalmanFilter kf_;
  double timestamp_;
  float tracking_time_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_CONE_H_
