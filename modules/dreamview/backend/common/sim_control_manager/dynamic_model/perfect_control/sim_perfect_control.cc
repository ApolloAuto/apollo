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

#include "modules/dreamview/backend/common/sim_control_manager/dynamic_model/perfect_control/sim_perfect_control.h"

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::Header;
using apollo::common::Point3D;
using apollo::common::Quaternion;
using apollo::common::TrajectoryPoint;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::InterpolateUsingLinearApproximation;
using apollo::common::math::InverseQuaternionRotate;
using apollo::common::util::FillHeader;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::planning::PlanningCommand;
using apollo::prediction::PredictionObstacles;
using apollo::relative_map::NavigationInfo;
using Json = nlohmann::json;

namespace {

void TransformToVRF(const Point3D &point_mrf, const Quaternion &orientation,
                    Point3D *point_vrf) {
  Eigen::Vector3d v_mrf(point_mrf.x(), point_mrf.y(), point_mrf.z());
  auto v_vrf = InverseQuaternionRotate(orientation, v_mrf);
  point_vrf->set_x(v_vrf.x());
  point_vrf->set_y(v_vrf.y());
  point_vrf->set_z(v_vrf.z());
}

bool IsSameHeader(const Header &lhs, const Header &rhs) {
  return lhs.sequence_num() == rhs.sequence_num() &&
         lhs.timestamp_sec() == rhs.timestamp_sec();
}

}  // namespace

SimPerfectControl::SimPerfectControl(const MapService *map_service)
    : SimControlBase(),
      map_service_(map_service),
      node_(cyber::CreateNode("sim_perfect_control")),
      current_trajectory_(std::make_shared<ADCTrajectory>()) {
  InitTimerAndIO();
}

void SimPerfectControl::InitTimerAndIO() {
  localization_reader_ =
      node_->CreateReader<LocalizationEstimate>(FLAGS_localization_topic);
  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<ADCTrajectory> &trajectory) {
        this->OnPlanning(trajectory);
      });
  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      FLAGS_planning_command,
      [this](const std::shared_ptr<PlanningCommand> &planning_command) {
        this->OnPlanningCommand(planning_command);
      });
  navigation_reader_ = node_->CreateReader<NavigationInfo>(
      FLAGS_navigation_topic,
      [this](const std::shared_ptr<NavigationInfo> &navigation_info) {
        this->OnReceiveNavigationInfo(navigation_info);
      });
  prediction_reader_ = node_->CreateReader<PredictionObstacles>(
      FLAGS_prediction_topic,
      [this](const std::shared_ptr<PredictionObstacles> &obstacles) {
        this->OnPredictionObstacles(obstacles);
      });

  localization_writer_ =
      node_->CreateWriter<LocalizationEstimate>(FLAGS_localization_topic);
  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);
  prediction_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);

  // Start timer to publish localization and chassis messages.
  sim_control_timer_.reset(new cyber::Timer(
      kSimControlIntervalMs, [this]() { this->RunOnce(); }, false));
  sim_prediction_timer_.reset(new cyber::Timer(
      kSimPredictionIntervalMs, [this]() { this->PublishDummyPrediction(); },
      false));
}

void SimPerfectControl::Init(bool set_start_point,
                             nlohmann::json start_point_attr,
                             bool use_start_point_position) {
  if (set_start_point && !FLAGS_use_navigation_mode) {
    InitStartPoint(start_point_attr["start_velocity"],
                   start_point_attr["start_acceleration"]);
  }
}

void SimPerfectControl::InitStartPoint(double x, double y,
                                       double start_velocity,
                                       double start_acceleration) {
  TrajectoryPoint point;
  // Use the scenario start point as start point,
  start_point_from_localization_ = false;
  point.mutable_path_point()->set_x(x);
  point.mutable_path_point()->set_y(y);
  // z use default 0
  point.mutable_path_point()->set_z(0);
  double theta = 0.0;
  double s = 0.0;
  map_service_->GetPoseWithRegardToLane(x, y, &theta, &s);
  point.mutable_path_point()->set_theta(theta);
  point.set_v(start_velocity);
  point.set_a(start_acceleration);
  SetStartPoint(point);
}

void SimPerfectControl::ReSetPoinstion(double x, double y, double heading) {
  std::lock_guard<std::mutex> lock(mutex_);
  TrajectoryPoint point;
  point.mutable_path_point()->set_x(x);
  point.mutable_path_point()->set_y(y);
  // z use default 0
  point.mutable_path_point()->set_z(0);
  point.mutable_path_point()->set_theta(heading);
  SetStartPoint(point);
  InternalReset();
}

void SimPerfectControl::InitStartPoint(double start_velocity,
                                       double start_acceleration) {                               
  TrajectoryPoint point;
  // Use the latest localization position as start point,
  // fall back to a dummy point from map
  localization_reader_->Observe();
  start_point_from_localization_ = false;
  if (!localization_reader_->Empty()) {
    const auto &localization = localization_reader_->GetLatestObserved();
    const auto &pose = localization->pose();
    if (map_service_->PointIsValid(pose.position().x(), pose.position().y())) {
      point.mutable_path_point()->set_x(pose.position().x());
      point.mutable_path_point()->set_y(pose.position().y());
      point.mutable_path_point()->set_z(pose.position().z());
      point.mutable_path_point()->set_theta(pose.heading());
      point.set_v(
          std::hypot(pose.linear_velocity().x(), pose.linear_velocity().y()));
      // Calculates the dot product of acceleration and velocity. The sign
      // of this projection indicates whether this is acceleration or
      // deceleration.
      double projection =
          pose.linear_acceleration().x() * pose.linear_velocity().x() +
          pose.linear_acceleration().y() * pose.linear_velocity().y();

      // Calculates the magnitude of the acceleration. Negate the value if
      // it is indeed a deceleration.
      double magnitude = std::hypot(pose.linear_acceleration().x(),
                                    pose.linear_acceleration().y());
      point.set_a(std::signbit(projection) ? -magnitude : magnitude);
      start_point_from_localization_ = true;
    }
  }
  if (!start_point_from_localization_) {
    apollo::common::PointENU start_point;
    if (!map_service_->GetStartPoint(&start_point)) {
      AWARN << "Failed to get a dummy start point from map!";
      return;
    }
    point.mutable_path_point()->set_x(start_point.x());
    point.mutable_path_point()->set_y(start_point.y());
    point.mutable_path_point()->set_z(start_point.z());
    double theta = 0.0;
    double s = 0.0;
    map_service_->GetPoseWithRegardToLane(start_point.x(), start_point.y(),
                                          &theta, &s);
    point.mutable_path_point()->set_theta(theta);
    point.set_v(start_velocity);
    point.set_a(start_acceleration);
  }
  SetStartPoint(point);
}

void SimPerfectControl::SetStartPoint(const TrajectoryPoint &start_point) {
  next_point_ = start_point;
  prev_point_index_ = next_point_index_ = 0;
  received_planning_ = false;
}

void SimPerfectControl::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  InternalReset();
}

void SimPerfectControl::Stop() {
  if (enabled_) {
    {
      std::lock_guard<std::mutex> lock(timer_mutex_);
      sim_control_timer_->Stop();
      sim_prediction_timer_->Stop();
    }
    enabled_ = false;
  }
}

void SimPerfectControl::InternalReset() {
  current_routing_header_.Clear();
  send_dummy_prediction_ = true;
  ClearPlanning();
}

void SimPerfectControl::ClearPlanning() {
  current_trajectory_->Clear();
  received_planning_ = false;
}

void SimPerfectControl::OnReceiveNavigationInfo(
    const std::shared_ptr<NavigationInfo> &navigation_info) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  if (navigation_info->navigation_path_size() > 0) {
    const auto &path = navigation_info->navigation_path(0).path();
    if (path.path_point_size() > 0) {
      adc_position_ = path.path_point(0);
    }
  }
}

void SimPerfectControl::OnPlanningCommand(
    const std::shared_ptr<PlanningCommand> &planning_command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  // Avoid not sending prediction messages to planning after playing packets
  // with obstacles
  send_dummy_prediction_ = true;
  current_routing_header_ = planning_command->header();
  AINFO << "planning_command: " << planning_command->DebugString();

  // if (planning_command->lane_follow_command()
  //         .routing_request()
  //         .is_start_pose_set()) {
  //   auto lane_follow_command = planning_command->mutable_lane_follow_command();
  //   const auto &start_pose =
  //       lane_follow_command->routing_request().waypoint(0).pose();
  //   ClearPlanning();
  //   TrajectoryPoint point;
  //   point.mutable_path_point()->set_x(start_pose.x());
  //   point.mutable_path_point()->set_y(start_pose.y());
  //   point.set_a(next_point_.has_a() ? next_point_.a() : 0.0);
  //   point.set_v(next_point_.has_v() ? next_point_.v() : 0.0);
  //   double theta = 0.0;
  //   double s = 0.0;

  //   // Find the lane nearest to the start pose and get its heading as theta.
  //   map_service_->GetPoseWithRegardToLane(start_pose.x(), start_pose.y(),
  //                                         &theta, &s);

  //   point.mutable_path_point()->set_theta(theta);
  //   SetStartPoint(point);
  // }
  return;
}

void SimPerfectControl::OnPredictionObstacles(
    const std::shared_ptr<PredictionObstacles> &obstacles) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_) {
    return;
  }

  send_dummy_prediction_ = obstacles->header().module_name() == "SimPrediction";
}

void SimPerfectControl::Start() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_) {
    // When there is no localization yet, Init(true) will use a
    // dummy point from the current map as an arbitrary start.
    // When localization is already available, we do not need to
    // reset/override the start point.
    localization_reader_->Observe();
    Json start_point_attr({});
    start_point_attr["start_velocity"] =
        next_point_.has_v() ? next_point_.v() : 0.0;
    start_point_attr["start_acceleration"] =
        next_point_.has_a() ? next_point_.a() : 0.0;
    Init(true, start_point_attr);
    InternalReset();
    {
      std::lock_guard<std::mutex> lock(timer_mutex_);
      sim_control_timer_->Start();
      sim_prediction_timer_->Start();
    }
    enabled_ = true;
  }
}

void SimPerfectControl::Start(double x, double y, double v, double a) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    // Do not use localization info. use scenario start point to init start
    // point.
    InitStartPoint(x, y, v, a);
    InternalReset();
    sim_control_timer_->Start();
    sim_prediction_timer_->Start();
    enabled_ = true;
  }
}

void SimPerfectControl::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_) {
    return;
  }

  // Reset current trajectory and the indices upon receiving a new trajectory.
  // The routing SimPerfectControl owns must match with the one Planning has.
  if (IsSameHeader(trajectory->routing_header(), current_routing_header_)) {
    current_trajectory_ = trajectory;
    prev_point_index_ = 0;
    next_point_index_ = 0;
    received_planning_ = true;
  } else {
    ClearPlanning();
  }
}

void SimPerfectControl::Freeze() {
  next_point_.set_v(0.0);
  next_point_.set_a(0.0);
  prev_point_ = next_point_;
}

void SimPerfectControl::RunOnce() {
  std::lock_guard<std::mutex> lock(mutex_);

  TrajectoryPoint trajectory_point;
  Chassis::GearPosition gear_position;
  if (!PerfectControlModel(&trajectory_point, &gear_position)) {
    AERROR << "Failed to calculate next point with perfect control model";
    return;
  }

  PublishChassis(trajectory_point.v(), gear_position);
  PublishLocalization(trajectory_point);
}

bool SimPerfectControl::PerfectControlModel(
    TrajectoryPoint *point, Chassis::GearPosition *gear_position) {
  // Result of the interpolation.
  auto current_time = Clock::NowInSeconds();
  const auto &trajectory = current_trajectory_->trajectory_point();
  *gear_position = current_trajectory_->gear();

  if (!received_planning_) {
    prev_point_ = next_point_;
  } else {
    if (current_trajectory_->estop().is_estop()) {
      // Freeze the car when there's an estop or the current trajectory has
      // been exhausted.
      Freeze();
    } else if (next_point_index_ >= trajectory.size()) {
       prev_point_ = next_point_;
    } else {
      // Determine the status of the car based on received planning message.
      while (next_point_index_ < trajectory.size() &&
             current_time > trajectory.Get(next_point_index_).relative_time() +
                                current_trajectory_->header().timestamp_sec()) {
        ++next_point_index_;
      }

      if (next_point_index_ >= trajectory.size()) {
        next_point_index_ = trajectory.size() - 1;
      }

      if (next_point_index_ == 0) {
        AERROR << "First trajectory point is a future point!";
        return false;
      }

      prev_point_index_ = next_point_index_ - 1;

      next_point_ = trajectory.Get(next_point_index_);
      prev_point_ = trajectory.Get(prev_point_index_);
    }
  }

  if (current_time > next_point_.relative_time() +
                         current_trajectory_->header().timestamp_sec()) {
    // Don't try to extrapolate if relative_time passes last point
    *point = next_point_;
  } else {
    *point = InterpolateUsingLinearApproximation(
        prev_point_, next_point_,
        current_time - current_trajectory_->header().timestamp_sec());
  }
  return true;
}

void SimPerfectControl::PublishChassis(double cur_speed,
                                       Chassis::GearPosition gear_position) {
  auto chassis = std::make_shared<Chassis>();
  FillHeader("SimPerfectControl", chassis.get());

  chassis->set_engine_started(true);
  chassis->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis->set_gear_location(gear_position);

  chassis->set_speed_mps(static_cast<float>(cur_speed));
  if (gear_position == canbus::Chassis::GEAR_REVERSE) {
    chassis->set_speed_mps(-chassis->speed_mps());
  }

  chassis->set_throttle_percentage(0.0);
  chassis->set_brake_percentage(0.0);

  chassis_writer_->Write(chassis);
}

void SimPerfectControl::PublishLocalization(const TrajectoryPoint &point) {
  auto localization = std::make_shared<LocalizationEstimate>();
  FillHeader("SimPerfectControl", localization.get());
  auto *pose = localization->mutable_pose();
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

    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({flu_x, flu_y}, cur_theta);

    enu_coordinate.x() += adc_position_.x();
    enu_coordinate.y() += adc_position_.y();
    pose->mutable_position()->set_x(enu_coordinate.x());
    pose->mutable_position()->set_y(enu_coordinate.y());
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
  auto *linear_acceleration = pose->mutable_linear_acceleration();
  linear_acceleration->set_x(std::cos(cur_theta) * point.a());
  linear_acceleration->set_y(std::sin(cur_theta) * point.a());
  linear_acceleration->set_z(0);

  TransformToVRF(pose->linear_acceleration(), pose->orientation(),
                 pose->mutable_linear_acceleration_vrf());
  localization_writer_->Write(localization);

  adc_position_.set_x(pose->position().x());
  adc_position_.set_y(pose->position().y());
  adc_position_.set_z(pose->position().z());
}

void SimPerfectControl::PublishDummyPrediction() {
  auto prediction = std::make_shared<PredictionObstacles>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!send_dummy_prediction_) {
      return;
    }
    FillHeader("SimPrediction", prediction.get());
  }
  prediction_writer_->Write(prediction);
}

}  // namespace dreamview
}  // namespace apollo
