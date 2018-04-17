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

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::Header;
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

bool IsSameHeader(const Header& lhs, const Header& rhs) {
  return lhs.sequence_num() == rhs.sequence_num() &&
         lhs.timestamp_sec() == rhs.timestamp_sec();
}

}  // namespace

SimControl::SimControl(const MapService* map_service)
    : map_service_(map_service) {}

void SimControl::Init(bool set_start_point, double start_velocity,
                      double start_acceleration) {
  // Setup planning and routing result data callback.
  AdapterManager::AddPlanningCallback(&SimControl::OnPlanning, this);
  AdapterManager::AddRoutingResponseCallback(&SimControl::OnRoutingResponse,
                                             this);
  AdapterManager::AddNavigationCallback(&SimControl::OnReceiveNavigationInfo,
                                        this);

  // Start timer to publish localization and chassis messages.
  sim_control_timer_ = AdapterManager::CreateTimer(
      ros::Duration(kSimControlInterval), &SimControl::TimerCallback, this);

  if (set_start_point && !FLAGS_use_navigation_mode) {
    apollo::common::PointENU start_point;
    if (!map_service_->GetStartPoint(&start_point)) {
      AWARN << "Failed to get a dummy start point from map!";
      return;
    }
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(start_point.x());
    point.mutable_path_point()->set_y(start_point.y());
    point.mutable_path_point()->set_z(start_point.z());
    point.set_v(start_velocity);
    point.set_a(start_acceleration);
    SetStartPoint(point);
  }

  start_velocity_ = start_velocity;
  start_acceleration_ = start_acceleration;
}

void SimControl::OnReceiveNavigationInfo(
    const relative_map::NavigationInfo& navigation_info) {
  navigation_info_ = navigation_info;
  if (navigation_info_.navigation_path_size() > 0) {
    const auto& path = navigation_info_.navigation_path(0).path();
    if (path.path_point_size() > 0) {
      adc_position_ = path.path_point(0);
    }
  }
}

void SimControl::SetStartPoint(const TrajectoryPoint& start_point) {
  next_point_ = start_point;
  prev_point_index_ = next_point_index_ = 0;
  received_planning_ = false;
}

void SimControl::ClearPlanning() {
  current_trajectory_.Clear();
  received_planning_ = false;
  if (planning_count_ > 0) {
    planning_count_ = 0;
  }
}

void SimControl::Reset() {
  current_routing_header_.Clear();
  re_routing_triggered_ = false;
  ClearPlanning();
}

void SimControl::OnRoutingResponse(const RoutingResponse& routing) {
  if (!enabled_) {
    return;
  }

  CHECK_GE(routing.routing_request().waypoint_size(), 2)
      << "routing should have at least two waypoints";
  const auto& start_pose = routing.routing_request().waypoint(0).pose();

  current_routing_header_ = routing.header();

  // If this is from a planning re-routing request, don't reset car's location.
  re_routing_triggered_ =
      routing.routing_request().header().module_name() == "planning";
  if (!re_routing_triggered_) {
    ClearPlanning();
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(start_pose.x());
    point.mutable_path_point()->set_y(start_pose.y());
    point.set_a(0.0);
    point.set_v(0.0);
    double theta = 0.0;
    double s = 0.0;
    map_service_->GetPoseWithRegardToLane(start_pose.x(), start_pose.y(),
                                          &theta, &s);
    point.mutable_path_point()->set_theta(theta);
    SetStartPoint(point);
  }
}

void SimControl::Start() {
  if (!enabled_) {
    if (!inited_) {
      // Only place the car when there is not a localization.
      Init(AdapterManager::GetLocalization()->Empty());
    }
    Reset();
    sim_control_timer_.start();
    enabled_ = true;
  }
}

void SimControl::Stop() {
  if (enabled_) {
    sim_control_timer_.stop();
    enabled_ = false;
  }
}

void SimControl::OnPlanning(const apollo::planning::ADCTrajectory& trajectory) {
  if (!enabled_) {
    return;
  }

  // Reset current trajectory and the indices upon receiving a new trajectory.
  // The routing SimControl owns must match with the one Planning has.
  if (re_routing_triggered_ ||
      IsSameHeader(trajectory.routing_header(), current_routing_header_)) {
    // Hold a few cycles until the position information is fully refreshed on
    // planning side. Don't wait for the very first planning received.
    ++planning_count_;
    if (planning_count_ == 0 || planning_count_ >= kPlanningCountToStart) {
      planning_count_ = kPlanningCountToStart;
      current_trajectory_ = trajectory;
      prev_point_index_ = 0;
      next_point_index_ = 0;
      received_planning_ = true;
    }
  } else {
    ClearPlanning();
  }
}

void SimControl::Freeze() {
  next_point_.set_v(0.0);
  next_point_.set_a(0.0);
  prev_point_ = next_point_;
}

void SimControl::TimerCallback(const ros::TimerEvent& event) { RunOnce(); }

void SimControl::RunOnce() {
  TrajectoryPoint trajectory_point;
  if (!PerfectControlModel(&trajectory_point)) {
    AERROR << "Failed to calculate next point with perfect control model";
    return;
  }
  PublishChassis(trajectory_point.v());
  PublishLocalization(trajectory_point);
}

bool SimControl::PerfectControlModel(TrajectoryPoint* point) {
  // Result of the interpolation.
  auto relative_time =
      Clock::NowInSeconds() - current_trajectory_.header().timestamp_sec();
  const auto& trajectory = current_trajectory_.trajectory_point();

  if (!received_planning_) {
    prev_point_ = next_point_;
  } else {
    if (current_trajectory_.estop().is_estop() ||
        next_point_index_ >= trajectory.size()) {
      // Freeze the car when there's an estop or the current trajectory has
      // been exhausted.
      Freeze();
    } else {
      // Determine the status of the car based on received planning message.
      while (next_point_index_ < trajectory.size() &&
             relative_time >
                 trajectory.Get(next_point_index_).relative_time()) {
        ++next_point_index_;
      }

      if (next_point_index_ == 0) {
        AERROR << "First trajectory point is a future point!";
        return false;
      }

      if (next_point_index_ >= trajectory.size()) {
        next_point_index_ = trajectory.size() - 1;
      }
      prev_point_index_ = next_point_index_ - 1;

      next_point_ = trajectory.Get(next_point_index_);
      prev_point_ = trajectory.Get(prev_point_index_);
    }
  }
  *point = apollo::common::math::InterpolateUsingLinearApproximation(
      prev_point_, next_point_, relative_time);
  return true;
}

void SimControl::PublishChassis(double cur_speed) {
  Chassis chassis;
  AdapterManager::FillChassisHeader("SimControl", &chassis);

  chassis.set_engine_started(true);
  chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis.set_gear_location(Chassis::GEAR_DRIVE);

  chassis.set_speed_mps(cur_speed);
  chassis.set_throttle_percentage(0.0);
  chassis.set_brake_percentage(0.0);

  AdapterManager::PublishChassis(chassis);
}

void SimControl::PublishLocalization(const TrajectoryPoint& point) {
  LocalizationEstimate localization;
  AdapterManager::FillLocalizationHeader("SimControl", &localization);

  auto* pose = localization.mutable_pose();
  auto prev = prev_point_.path_point();
  auto next = next_point_.path_point();

  // Set position
  pose->mutable_position()->set_x(point.path_point().x());
  pose->mutable_position()->set_y(point.path_point().y());
  pose->mutable_position()->set_z(point.path_point().z());
  // Set orientation and heading
  double cur_theta = point.path_point().theta();

  if (FLAGS_use_navigation_mode) {
    double flu_x = point.path_point().x();
    double flu_y = point.path_point().y();
    double enu_x = 0.0;
    double enu_y = 0.0;
    common::math::RotateAxis(-cur_theta, flu_x, flu_y, &enu_x, &enu_y);
    enu_x += adc_position_.x();
    enu_y += adc_position_.y();
    pose->mutable_position()->set_x(enu_x);
    pose->mutable_position()->set_y(enu_y);
  }

  Eigen::Quaternion<double> cur_orientation =
      HeadingToQuaternion<double>(cur_theta);
  pose->mutable_orientation()->set_qw(cur_orientation.w());
  pose->mutable_orientation()->set_qx(cur_orientation.x());
  pose->mutable_orientation()->set_qy(cur_orientation.y());
  pose->mutable_orientation()->set_qz(cur_orientation.z());
  pose->set_heading(cur_theta);

  // Set linear_velocity
  pose->mutable_linear_velocity()->set_x(std::cos(cur_theta) * point.v());
  pose->mutable_linear_velocity()->set_y(std::sin(cur_theta) * point.v());
  pose->mutable_linear_velocity()->set_z(0);

  // Set angular_velocity in both map reference frame and vehicle reference
  // frame
  pose->mutable_angular_velocity()->set_x(0);
  pose->mutable_angular_velocity()->set_y(0);
  pose->mutable_angular_velocity()->set_z(point.v() *
                                          point.path_point().kappa());

  TransformToVRF(pose->angular_velocity(), pose->orientation(),
                 pose->mutable_angular_velocity_vrf());

  // Set linear_acceleration in both map reference frame and vehicle reference
  // frame
  auto* linear_acceleration = pose->mutable_linear_acceleration();
  linear_acceleration->set_x(std::cos(cur_theta) * point.a());
  linear_acceleration->set_y(std::sin(cur_theta) * point.a());
  linear_acceleration->set_z(0);

  TransformToVRF(pose->linear_acceleration(), pose->orientation(),
                 pose->mutable_linear_acceleration_vrf());

  AdapterManager::PublishLocalization(localization);

  adc_position_.set_x(pose->position().x());
  adc_position_.set_y(pose->position().y());
  adc_position_.set_z(pose->position().z());
}

}  // namespace dreamview
}  // namespace apollo
