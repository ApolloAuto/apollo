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

#include "modules/dreamview/backend/sim_control/sim_control.h"

#include <cmath>

#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::Point3D;
using apollo::common::Quaternion;
using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterManager;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::InverseQuaternionRotate;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::QuaternionToHeading;
using apollo::common::time::Clock;
using apollo::common::util::GetProtoFromFile;
using apollo::localization::LocalizationEstimate;
using apollo::routing::RoutingResponse;

namespace {

void TransformToVRF(const Point3D& point_mrf, const Quaternion& orientation,
                    Point3D* point_vrf) {
  Eigen::Vector3d v_mrf(point_mrf.x(), point_mrf.y(), point_mrf.z());
  auto v_vrf = InverseQuaternionRotate(orientation, v_mrf);
  point_vrf->set_x(v_vrf.x());
  point_vrf->set_y(v_vrf.y());
  point_vrf->set_z(v_vrf.z());
}

}  // namespace

SimControl::SimControl(const MapService* map_service)
    : map_service_(map_service),
      prev_point_index_(0),
      next_point_index_(0),
      received_planning_(false),
      enabled_(FLAGS_enable_sim_control) {}

void SimControl::Init(bool set_start_point) {
  // Setup planning and routing result data callback.
  AdapterManager::AddPlanningCallback(&SimControl::OnPlanning, this);
  AdapterManager::AddRoutingResponseCallback(&SimControl::OnRoutingResponse,
                                             this);

  // Start timer to publish localization and chassis messages.
  sim_control_timer_ = AdapterManager::CreateTimer(
      ros::Duration(kSimControlInterval), &SimControl::TimerCallback, this);

  if (set_start_point) {
    apollo::common::PointENU start_point;
    if (!map_service_->GetStartPoint(&start_point)) {
      AWARN << "Failed to get a dummy start point from map!";
      return;
    }
    SetStartPoint(start_point.x(), start_point.y());
  }
}

void SimControl::SetStartPoint(const double x, const double y) {
  next_point_.set_v(0.0);
  next_point_.set_a(0.0);

  auto* next_point = next_point_.mutable_path_point();
  next_point->set_x(x);
  next_point->set_y(y);
  next_point->set_z(0.0);

  double theta = 0.0;
  double s = 0.0;
  if (!map_service_->GetPoseWithRegardToLane(next_point->x(), next_point->y(),
                                             &theta, &s)) {
    AERROR << "Failed to get heading from map! Treat theta and s as 0.0!";
  }
  next_point->set_theta(theta);
  next_point->set_s(s);
  next_point->set_kappa(0.0);

  prev_point_index_ = next_point_index_ = 0;
  received_planning_ = false;

  Start();
}

void SimControl::OnRoutingResponse(const RoutingResponse& routing) {
  DCHECK_LE(2, routing.routing_request().waypoint_size());
  const auto& start_pose = routing.routing_request().waypoint(0).pose();
  SetStartPoint(start_pose.x(), start_pose.y());
}

void SimControl::Start() {
  if (enabled_) {
    sim_control_timer_.start();
  }
}

void SimControl::Stop() {
  sim_control_timer_.stop();
}

void SimControl::OnPlanning(const apollo::planning::ADCTrajectory& trajectory) {
  // Reset current trajectory and the indices upon receiving a new trajectory.
  current_trajectory_ = trajectory;
  prev_point_index_ = 0;
  next_point_index_ = 0;
  received_planning_ = true;
}

void SimControl::Freeze() {
  next_point_.set_v(0.0);
  next_point_.set_a(0.0);
  prev_point_ = next_point_;
}

double SimControl::AbsoluteTimeOfNextPoint() {
  return current_trajectory_.header().timestamp_sec() +
         current_trajectory_.trajectory_point(next_point_index_)
             .relative_time();
}

bool SimControl::NextPointWithinRange() {
  return next_point_index_ < current_trajectory_.trajectory_point_size() - 1;
}

void SimControl::TimerCallback(const ros::TimerEvent& event) {
  RunOnce();
}

void SimControl::RunOnce() {
  // Result of the interpolation.
  double lambda = 0;
  auto current_time = Clock::NowInSecond();

  if (!received_planning_) {
    prev_point_ = next_point_;
  } else {
    if (current_trajectory_.estop().is_estop() || !NextPointWithinRange()) {
      // Freeze the car when there's an estop or the current trajectory has been
      // exhausted.
      Freeze();
    } else {
      // Determine the status of the car based on received planning message.
      double timestamp = current_trajectory_.header().timestamp_sec();

      while (NextPointWithinRange() &&
             current_time > AbsoluteTimeOfNextPoint()) {
        ++next_point_index_;
      }

      if (next_point_index_ == 0) {
        AERROR << "First trajectory point is a future point!";
        return;
      }

      if (current_time > AbsoluteTimeOfNextPoint()) {
        prev_point_index_ = next_point_index_;
      } else {
        prev_point_index_ = next_point_index_ - 1;
      }

      next_point_ = current_trajectory_.trajectory_point(next_point_index_);
      prev_point_ = current_trajectory_.trajectory_point(prev_point_index_);

      // Calculate the ratio based on the the position of current time in
      // between the previous point and the next point, where lambda =
      // (current_point - prev_point) / (next_point - prev_point).
      if (next_point_index_ != prev_point_index_) {
        lambda = (current_time - timestamp - prev_point_.relative_time()) /
                 (next_point_.relative_time() - prev_point_.relative_time());
      }
    }
  }

  PublishChassis(lambda);
  PublishLocalization(lambda);
}

void SimControl::PublishChassis(double lambda) {
  Chassis chassis;
  AdapterManager::FillChassisHeader("SimControl", &chassis);

  chassis.set_engine_started(true);
  chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis.set_gear_location(Chassis::GEAR_DRIVE);

  double cur_speed = Interpolate(prev_point_.v(), next_point_.v(), lambda);
  chassis.set_speed_mps(cur_speed);
  chassis.set_throttle_percentage(0.0);
  chassis.set_brake_percentage(0.0);

  AdapterManager::PublishChassis(chassis);
}

void SimControl::PublishLocalization(double lambda) {
  LocalizationEstimate localization;
  AdapterManager::FillLocalizationHeader("SimControl", &localization);

  auto* pose = localization.mutable_pose();
  auto prev = prev_point_.path_point();
  auto next = next_point_.path_point();

  // Set position
  double cur_x = Interpolate(prev.x(), next.x(), lambda);
  pose->mutable_position()->set_x(cur_x);
  double cur_y = Interpolate(prev.y(), next.y(), lambda);
  pose->mutable_position()->set_y(cur_y);
  double cur_z = Interpolate(prev.z(), next.z(), lambda);
  pose->mutable_position()->set_z(cur_z);

  // Set orientation and heading
  double cur_theta = NormalizeAngle(
      prev.theta() + lambda * NormalizeAngle(next.theta() - prev.theta()));

  Eigen::Quaternion<double> cur_orientation =
      HeadingToQuaternion<double>(cur_theta);
  pose->mutable_orientation()->set_qw(cur_orientation.w());
  pose->mutable_orientation()->set_qx(cur_orientation.x());
  pose->mutable_orientation()->set_qy(cur_orientation.y());
  pose->mutable_orientation()->set_qz(cur_orientation.z());
  pose->set_heading(cur_theta);

  // Set linear_velocity
  double cur_speed = Interpolate(prev_point_.v(), next_point_.v(), lambda);
  pose->mutable_linear_velocity()->set_x(std::cos(cur_theta) * cur_speed);
  pose->mutable_linear_velocity()->set_y(std::sin(cur_theta) * cur_speed);
  pose->mutable_linear_velocity()->set_z(0);

  // Set angular_velocity in both map reference frame and vehicle reference
  // frame
  double cur_curvature = Interpolate(prev.kappa(), next.kappa(), lambda);
  pose->mutable_angular_velocity()->set_x(0);
  pose->mutable_angular_velocity()->set_y(0);
  pose->mutable_angular_velocity()->set_z(cur_speed * cur_curvature);

  TransformToVRF(pose->angular_velocity(), pose->orientation(),
                 pose->mutable_angular_velocity_vrf());

  // Set linear_acceleration in both map reference frame and vehicle reference
  // frame
  double cur_acceleration_s =
      Interpolate(prev_point_.a(), next_point_.a(), lambda);
  auto* linear_acceleration = pose->mutable_linear_acceleration();
  linear_acceleration->set_x(std::cos(cur_theta) * cur_acceleration_s);
  linear_acceleration->set_y(std::sin(cur_theta) * cur_acceleration_s);
  linear_acceleration->set_z(0);

  TransformToVRF(pose->linear_acceleration(), pose->orientation(),
                 pose->mutable_linear_acceleration_vrf());

  AdapterManager::PublishLocalization(localization);
}

}  // namespace dreamview
}  // namespace apollo
