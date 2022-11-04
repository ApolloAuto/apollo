/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/sim_control_manager/core/sim_control_with_model_base.h"

#include "modules/dreamview/backend/map/map_service.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::Point3D;
using apollo::common::Quaternion;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::InterpolateUsingLinearApproximation;
using apollo::common::math::InverseQuaternionRotate;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::QuaternionToHeading;
using apollo::common::util::FillHeader;
using apollo::control::ControlCommand;
using apollo::localization::LocalizationEstimate;
using apollo::prediction::PredictionObstacles;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::sim_control::SimCarStatus;
using Json = nlohmann::json;

/**
 * @brief Construct a new Sim Control With Model Base:: Sim Control With Model
 * Base object
 * @param node_name: depens on different dynamic model
 */
SimControlWithModelBase::SimControlWithModelBase(const std::string& node_name)
    : node_(cyber::CreateNode(node_name)),
      gear_position_(0),
      dt_(0.01),
      map_service_(new MapService()) {
  InitTimerAndIO();
}

void SimControlWithModelBase::InitTimerAndIO() {
  // Setup  control result data callback.
  cyber::ReaderConfig control_cmd_reader_config;
  control_cmd_reader_config.channel_name = FLAGS_control_command_topic;
  control_cmd_reader_config.pending_queue_size =
      FLAGS_reader_pending_queue_size;
  prediction_reader_ = node_->CreateReader<PredictionObstacles>(
      FLAGS_prediction_topic,
      [this](const std::shared_ptr<PredictionObstacles>& obstacles) {
        this->OnPredictionObstacles(obstacles);
      });
  control_command_reader_ = node_->CreateReader<ControlCommand>(
      control_cmd_reader_config,
      [this](const std::shared_ptr<ControlCommand>& cmd) {
        ADEBUG << "Received control data: run canbus callback.";
        OnControlCommand(*cmd);
      });
  // Setup routing callback.
  cyber::ReaderConfig routing_reader_config;
  routing_reader_config.channel_name = FLAGS_routing_response_topic;
  routing_reader_config.pending_queue_size = FLAGS_reader_pending_queue_size;
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      routing_reader_config,
      [this](const std::shared_ptr<RoutingResponse>& cmd) {
        ADEBUG << "Received routing data: run canbus callback.";
        OnRoutingResponse(*cmd);
      });
  routing_request_reader_ = node_->CreateReader<RoutingRequest>(
      FLAGS_routing_request_topic,
      [this](const std::shared_ptr<RoutingRequest>& routing_request) {
        this->OnRoutingRequest(routing_request);
      });

  // Setup localization callback.
  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_reader_pending_queue_size;
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);

  localization_writer_ =
      node_->CreateWriter<LocalizationEstimate>(FLAGS_localization_topic);
  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);
  prediction_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);

  // Start timer to publish localization and chassis messages.
  sim_control_timer_.reset(new cyber::Timer(
      kModelIntervalMs, [this]() { this->RunOnce(); }, false));
  sim_prediction_timer_.reset(new cyber::Timer(
      kSimPredictionIntervalMs, [this]() { this->PublishDummyPrediction(); },
      false));
}

void SimControlWithModelBase::UpdateGearPosition() {
  // update gear location, 1 drive, -1 reverse, 0 for other states
  if (control_cmd_.gear_location() == Chassis::GEAR_DRIVE) {
    gear_position_ = 1;
  } else if (control_cmd_.gear_location() == Chassis::GEAR_REVERSE) {
    gear_position_ = -1;
  } else {
    gear_position_ = 0;
  }
  current_point_.set_gear_position(gear_position_);
}

void SimControlWithModelBase::OnPredictionObstacles(
    const std::shared_ptr<PredictionObstacles>& obstacles) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  send_dummy_prediction_ =
      obstacles->header().module_name() == "SimDMPrediction";
}

void SimControlWithModelBase::Start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    InternalReset();
    // todo: Compatible with scenario conf and default value
    Json start_point_attr({});
    // start_point_attr:from scenario conf json or default value.here dm is
    // divided from scenario use default value
    start_point_attr["start_velocity"] = 0.0;
    start_point_attr["start_acceleration"] = 0.0;
    start_point_attr["start_heading"] = std::numeric_limits<double>::max();
    Init(true, start_point_attr);
    sim_control_timer_->Start();
    sim_prediction_timer_->Start();
    enabled_ = true;
  }
}

void SimControlWithModelBase::Start(double x, double y) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    InternalReset();
    Json start_point_attr({});
    start_point_attr["start_velocity"] = 0.0;
    start_point_attr["start_acceleration"] = 0.0;
    start_point_attr["start_heading"] = std::numeric_limits<double>::max();
    start_point_attr["x"] = x;
    start_point_attr["y"] = y;
    Init(true, start_point_attr, true);
    sim_control_timer_->Start();
    sim_prediction_timer_->Start();
    enabled_ = true;
  }
}

void SimControlWithModelBase::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (enabled_) {
    sim_control_timer_->Stop();
    sim_prediction_timer_->Stop();
    enabled_ = false;
  }
}

void SimControlWithModelBase::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  InternalReset();
}

void SimControlWithModelBase::InternalReset() {
  re_routing_triggered_ = false;
  current_routing_header_.Clear();
  start_auto_ = false;
  send_dummy_prediction_ = true;
}

void SimControlWithModelBase::OnControlCommand(
    const ControlCommand& control_cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  control_cmd_ = control_cmd;
}

void SimControlWithModelBase::OnRoutingResponse(
    const RoutingResponse& routing) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }

  CHECK_GE(routing.routing_request().waypoint_size(), 2)
      << "routing should have at least two waypoints";

  current_routing_header_ = routing.header();
  // If this is from a planning re-routing request, or the start point has been
  // initialized by an actual localization pose, don't reset the start point.
  // re_routing_triggered_ =
  //     routing.routing_request().header().module_name() == "planning";
  // if (!re_routing_triggered_ && !start_point_from_localization_) {
  //   CHECK_GE(routing.routing_request().waypoint_size(), 2)
  //       << "routing should have at least two waypoints";
  //   current_routing_header_ = routing.header();
  //   const auto& start_pose = routing.routing_request().waypoint(0).pose();
  //   SimCarStatus point;
  //   point.set_x(start_pose.x());
  //   point.set_y(start_pose.y());
  //   point.set_acceleration_s(start_acceleration_);
  //   point.set_speed(start_velocity_);
  //   // Use configured heading if available, otherwise find heading based on
  //   // first lane in routing response
  //   double theta = 0.0;
  //   if (start_heading_ < std::numeric_limits<double>::max()) {
  //     theta = start_heading_;
  //   } else if (routing.road_size() > 0) {
  //     auto start_lane = routing.road(0).passage(0).segment(0);
  //     theta =
  //         map_service_->GetLaneHeading(start_lane.id(),
  //         start_lane.start_s());
  //   }
  //   point.set_theta(theta);
  //   SetStartPoint(point);
  // }
}

void SimControlWithModelBase::OnRoutingRequest(
    const std::shared_ptr<RoutingRequest>& routing_request) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }

  CHECK_GE(routing_request->waypoint_size(), 2)
      << "routing should have at least two waypoints";
  const auto& start_pose = routing_request->waypoint(0).pose();

  SimCarStatus point;
  point.set_x(start_pose.x());
  point.set_y(start_pose.y());
  point.set_acceleration_s(start_acceleration_);
  point.set_speed(start_velocity_);
  // Use configured heading if available, otherwise find heading based on
  // first lane in routing response
  double theta = 0.0;
  double s = 0.0;
  const auto& start_way_point = routing_request->waypoint().Get(0);
  // If the lane id has been set, set theta as the lane heading.
  if (start_way_point.has_id()) {
    auto& hdmap = hdmap::HDMapUtil::BaseMap();
    hdmap::Id lane_id = hdmap::MakeMapId(start_way_point.id());
    auto lane = hdmap.GetLaneById(lane_id);
    if (nullptr != lane) {
      theta = lane->Heading(start_way_point.s());
    } else {
      map_service_->GetPoseWithRegardToLane(start_pose.x(), start_pose.y(),
                                            &theta, &s);
    }
  } else {
    // Find the lane nearest to the start pose and get its heading as theta.
    map_service_->GetPoseWithRegardToLane(start_pose.x(), start_pose.y(),
                                          &theta, &s);
  }
  point.set_theta(theta);
  // if (start_heading_ < std::numeric_limits<double>::max()) {
  //   theta = start_heading_;
  // } else if (routing_request.road_size() > 0) {
  //   auto start_lane = routing_request.road(0).passage(0).segment(0);
  //   theta = map_service_->GetLaneHeading(start_lane.id(),
  //   start_lane.start_s());
  // }
  // point.set_theta(theta);
  SetStartPoint(point);
}

void SimControlWithModelBase::SetStartPoint(const SimCarStatus& point) {
  previous_point_ = point;
}

void SimControlWithModelBase::InitStartPoint(nlohmann::json start_point_attr,
                                             bool use_start_point_position) {
  AINFO << "Init start point with position!";

  start_velocity_ = start_point_attr["start_velocity"];
  start_acceleration_ = start_point_attr["start_acceleration"];
  start_heading_ = start_point_attr["start_heading"];
  SimCarStatus point;
  localization_reader_->Observe();
  if (use_start_point_position) {
    // add start point position for dynamic model
    // new add feature to keep same with simcontrol
    start_point_from_localization_ = false;
    point.set_x(start_point_attr["x"]);
    point.set_y(start_point_attr["y"]);
    // z use default 0
    point.set_z(0);
    // Todo(@lijin): tmp not support map service,support theta
    // double theta = 0.0;
    // double s = 0.0;
    // map_service_->GetPoseWithRegardToLane(x, y, &theta, &s);
    // point.set_theta();
    point.set_speed(start_velocity_);
    point.set_acceleration_s(start_acceleration_);
  } else {
    if (localization_reader_->Empty()) {
      // Routing will provide all other pose info
      start_point_from_localization_ = false;
      point.set_speed(start_velocity_);
      point.set_acceleration_s(start_acceleration_);
    } else {
      start_point_from_localization_ = true;
      const auto& pose = localization_reader_->GetLatestObserved()->pose();

      point.set_x(pose.position().x());
      point.set_y(pose.position().y());
      point.set_z(pose.position().z());
      point.set_theta(pose.heading());
      point.set_speed(
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
      point.set_acceleration_s(std::signbit(projection) ? -magnitude
                                                        : magnitude);
      // Set init gear to neutral position
      point.set_gear_position(0);
    }
  }
  SetStartPoint(point);
}

/**
 * @brief pubish chassis info
 *
 * @param model_name
 */
void SimControlWithModelBase::PublishChassis(const std::string model_name) {
  auto chassis = std::make_shared<Chassis>();
  apollo::common::util::FillHeader(model_name, chassis.get());

  chassis->set_engine_started(true);
  chassis->set_speed_mps(current_point_.speed());
  chassis->set_odometer_m(current_point_.odometer());

  if (FLAGS_enable_steering_latency) {
    chassis->set_steering_percentage(filtered_control_cmd_.steering_target());
  } else {
    chassis->set_steering_percentage(control_cmd_.steering_target());
  }
  chassis->set_throttle_percentage(control_cmd_.throttle());
  chassis->set_brake_percentage(control_cmd_.brake());
  if (start_auto_) {
    chassis->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  } else {
    chassis->set_driving_mode(Chassis::COMPLETE_MANUAL);
  }
  chassis->set_gear_location(control_cmd_.gear_location());
  chassis_writer_->Write(chassis);
}

/**
 * @brief publish prediction info
 *
 */
void SimControlWithModelBase::PublishDummyPrediction() {
  auto prediction = std::make_shared<PredictionObstacles>();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!send_dummy_prediction_) {
      return;
    }
    FillHeader("SimDMPrediction", prediction.get());
  }
  prediction_writer_->Write(prediction);
}

void SimControlWithModelBase::PublishLocalization(
    const std::string model_name) {
  auto localization = std::make_shared<LocalizationEstimate>();
  FillHeader(model_name, localization.get());

  auto* pose = localization->mutable_pose();

  // Set position
  pose->mutable_position()->set_x(current_point_.x());
  pose->mutable_position()->set_y(current_point_.y());
  pose->mutable_position()->set_z(0.0);
  // Set orientation and heading
  double cur_theta = current_point_.theta();

  Eigen::Quaternion<double> cur_orientation =
      HeadingToQuaternion<double>(cur_theta);
  ADEBUG << "cur_theta" << cur_theta;
  ADEBUG << "cur_orientation_w" << cur_orientation.w() << "cur_orientation_x"
         << cur_orientation.x() << "cur_orientation_y" << cur_orientation.y()
         << "cur_orientation_z" << cur_orientation.z();
  pose->mutable_orientation()->set_qw(cur_orientation.w());
  pose->mutable_orientation()->set_qx(cur_orientation.x());
  pose->mutable_orientation()->set_qy(cur_orientation.y());
  pose->mutable_orientation()->set_qz(cur_orientation.z());
  pose->set_heading(cur_theta);

  // Set linear_velocity
  if (control_cmd_.gear_location() == Chassis::GEAR_REVERSE) {
    pose->mutable_linear_velocity()->set_x(std::cos(cur_theta) * -1.0 *
                                           current_point_.speed());
    pose->mutable_linear_velocity()->set_y(std::sin(cur_theta) * -1.0 *
                                           current_point_.speed());
  } else {
    pose->mutable_linear_velocity()->set_x(std::cos(cur_theta) *
                                           current_point_.speed());
    pose->mutable_linear_velocity()->set_y(std::sin(cur_theta) *
                                           current_point_.speed());
  }

  pose->mutable_linear_velocity()->set_z(0);

  // Set angular_velocity in both map reference frame and vehicle reference
  // frame
  double cur_curvature = std::tan(control_cmd_.steering_target() *
                                  vehicle_param_.max_steer_angle() / 100.0 /
                                  vehicle_param_.steer_ratio()) /
                         vehicle_param_.wheel_base();

  pose->mutable_angular_velocity()->set_x(0);
  pose->mutable_angular_velocity()->set_y(0);
  pose->mutable_angular_velocity()->set_z(current_point_.speed() *
                                          cur_curvature);

  TransformToVRF(pose->angular_velocity(), pose->orientation(),
                 pose->mutable_angular_velocity_vrf());

  // Set linear_acceleration in both map reference frame and vehicle reference
  // frame
  auto* linear_acceleration = pose->mutable_linear_acceleration();
  linear_acceleration->set_x(std::cos(cur_theta) *
                             current_point_.acceleration_s());
  linear_acceleration->set_y(std::sin(cur_theta) *
                             current_point_.acceleration_s());
  linear_acceleration->set_z(0);

  TransformToVRF(pose->linear_acceleration(), pose->orientation(),
                 pose->mutable_linear_acceleration_vrf());

  localization_writer_->Write(localization);

  adc_position_.set_x(pose->position().x());
  adc_position_.set_y(pose->position().y());
  adc_position_.set_z(pose->position().z());
}

}  // namespace dreamview
}  // namespace apollo
