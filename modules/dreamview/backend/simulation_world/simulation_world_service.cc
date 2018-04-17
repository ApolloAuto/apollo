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

#include "modules/dreamview/backend/simulation_world/simulation_world_service.h"

#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <vector>

#include "google/protobuf/util/json_util.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/proto/vehicle_signal.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/points_downsampler.h"
#include "modules/common/util/util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/util/trajectory_point_collector.h"
#include "modules/dreamview/proto/simulation_world.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::PathPoint;
using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessage;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::common::time::ToSecond;
using apollo::common::time::millis;
using apollo::common::util::DownsampleByAngle;
using apollo::common::util::GetProtoFromFile;
using apollo::control::ControlCommand;
using apollo::hdmap::Curve;
using apollo::hdmap::Map;
using apollo::hdmap::Path;
using apollo::localization::Gps;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::perception::TrafficLightDetection;
using apollo::planning::ADCTrajectory;
using apollo::planning::DecisionResult;
using apollo::planning::StopReasonCode;
using apollo::planning_internal::PlanningData;
using apollo::prediction::PredictionObstacle;
using apollo::prediction::PredictionObstacles;
using apollo::relative_map::MapMsg;
using apollo::relative_map::NavigationInfo;
using apollo::routing::RoutingResponse;

using Json = nlohmann::json;
using ::google::protobuf::util::MessageToJsonString;

// Angle threshold is about 5.72 degree.
static constexpr double kAngleThreshold = 0.1;

namespace {

double CalculateAcceleration(const Point3D &acceleration,
                             const Point3D &velocity) {
  // Calculates the dot product of acceleration and velocity. The sign
  // of this projection indicates whether this is acceleration or
  // deceleration.
  double projection =
      acceleration.x() * velocity.x() + acceleration.y() * velocity.y();

  // Calculates the magnitude of the acceleration. Negate the value if
  // it is indeed a deceleration.
  double magnitude = std::hypot(acceleration.x(), acceleration.y());
  return std::signbit(projection) ? -magnitude : magnitude;
}

Object::DisengageType DeduceDisengageType(const Chassis &chassis) {
  if (chassis.error_code() != Chassis::NO_ERROR) {
    return Object::DISENGAGE_CHASSIS_ERROR;
  }

  switch (chassis.driving_mode()) {
    case Chassis::COMPLETE_AUTO_DRIVE:
      return Object::DISENGAGE_NONE;
    case Chassis::COMPLETE_MANUAL:
      return Object::DISENGAGE_MANUAL;
    case Chassis::AUTO_STEER_ONLY:
      return Object::DISENGAGE_AUTO_STEER_ONLY;
    case Chassis::AUTO_SPEED_ONLY:
      return Object::DISENGAGE_AUTO_SPEED_ONLY;
    case Chassis::EMERGENCY_MODE:
      return Object::DISENGAGE_EMERGENCY;
    default:
      return Object::DISENGAGE_UNKNOWN;
  }
}

void SetObstacleType(const PerceptionObstacle &obstacle, Object *world_object) {
  if (world_object == nullptr) {
    return;
  }

  switch (obstacle.type()) {
    case PerceptionObstacle::UNKNOWN:
      world_object->set_type(Object_Type_UNKNOWN);
      break;
    case PerceptionObstacle::UNKNOWN_MOVABLE:
      world_object->set_type(Object_Type_UNKNOWN_MOVABLE);
      break;
    case PerceptionObstacle::UNKNOWN_UNMOVABLE:
      world_object->set_type(Object_Type_UNKNOWN_UNMOVABLE);
      break;
    case PerceptionObstacle::PEDESTRIAN:
      world_object->set_type(Object_Type_PEDESTRIAN);
      break;
    case PerceptionObstacle::BICYCLE:
      world_object->set_type(Object_Type_BICYCLE);
      break;
    case PerceptionObstacle::VEHICLE:
      world_object->set_type(Object_Type_VEHICLE);
      break;
    default:
      world_object->set_type(Object_Type_VIRTUAL);
  }
}

void SetStopReason(const StopReasonCode &reason_code, Decision *decision) {
  switch (reason_code) {
    case StopReasonCode::STOP_REASON_HEAD_VEHICLE:
      decision->set_stopreason(Decision::STOP_REASON_HEAD_VEHICLE);
      break;
    case StopReasonCode::STOP_REASON_DESTINATION:
      decision->set_stopreason(Decision::STOP_REASON_DESTINATION);
      break;
    case StopReasonCode::STOP_REASON_PEDESTRIAN:
      decision->set_stopreason(Decision::STOP_REASON_PEDESTRIAN);
      break;
    case StopReasonCode::STOP_REASON_OBSTACLE:
      decision->set_stopreason(Decision::STOP_REASON_OBSTACLE);
      break;
    case StopReasonCode::STOP_REASON_SIGNAL:
      decision->set_stopreason(Decision::STOP_REASON_SIGNAL);
      break;
    case StopReasonCode::STOP_REASON_STOP_SIGN:
      decision->set_stopreason(Decision::STOP_REASON_STOP_SIGN);
      break;
    case StopReasonCode::STOP_REASON_YIELD_SIGN:
      decision->set_stopreason(Decision::STOP_REASON_YIELD_SIGN);
      break;
    case StopReasonCode::STOP_REASON_CLEAR_ZONE:
      decision->set_stopreason(Decision::STOP_REASON_CLEAR_ZONE);
      break;
    case StopReasonCode::STOP_REASON_CROSSWALK:
      decision->set_stopreason(Decision::STOP_REASON_CROSSWALK);
      break;
    default:
      AWARN << "Unrecognizable stop reason code:" << reason_code;
  }
}

void UpdateTurnSignal(const apollo::common::VehicleSignal &signal,
                      Object *auto_driving_car) {
  if (signal.turn_signal() == apollo::common::VehicleSignal::TURN_LEFT) {
    auto_driving_car->set_current_signal("LEFT");
  } else if (signal.turn_signal() ==
             apollo::common::VehicleSignal::TURN_RIGHT) {
    auto_driving_car->set_current_signal("RIGHT");
  } else if (signal.emergency_light()) {
    auto_driving_car->set_current_signal("EMERGENCY");
  } else {
    auto_driving_car->set_current_signal("");
  }
}

void DownsampleCurve(Curve *curve) {
  auto *line_segment = curve->mutable_segment(0)->mutable_line_segment();
  std::vector<PointENU> points(line_segment->point().begin(),
                               line_segment->point().end());
  line_segment->clear_point();

  // Downsample points by angle then by distance.
  std::vector<int> sampled_indices = DownsampleByAngle(points, kAngleThreshold);
  for (int index : sampled_indices) {
    auto *point = line_segment->add_point();
    point->set_x(points[index].x());
    point->set_y(points[index].y());
  }
}

inline double SecToMs(const double sec) { return sec * 1000.0; }

}  // namespace

constexpr int SimulationWorldService::kMaxMonitorItems;

SimulationWorldService::SimulationWorldService(const MapService *map_service,
                                               bool routing_from_file)
    : map_service_(map_service),
      monitor_logger_(MonitorMessageItem::SIMULATOR),
      ready_to_push_(false) {
  RegisterMessageCallbacks();
  if (routing_from_file) {
    ReadRoutingFromFile(FLAGS_routing_response_file);
  }

  // Populate vehicle parameters.
  Object *auto_driving_car = world_.mutable_auto_driving_car();
  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  auto_driving_car->set_height(vehicle_param.height());
  auto_driving_car->set_width(vehicle_param.width());
  auto_driving_car->set_length(vehicle_param.length());
}

void SimulationWorldService::Update() {
  if (to_clear_) {
    // Clears received data.
    AdapterManager::GetChassis()->ClearData();
    AdapterManager::GetGps()->ClearData();
    AdapterManager::GetLocalization()->ClearData();
    AdapterManager::GetPerceptionObstacles()->ClearData();
    AdapterManager::GetPlanning()->ClearData();
    AdapterManager::GetPrediction()->ClearData();
    AdapterManager::GetRoutingResponse()->ClearData();
    AdapterManager::GetMonitor()->ClearData();

    // Clears simulation world except for the car information.
    auto car = world_.auto_driving_car();
    world_.Clear();
    *world_.mutable_auto_driving_car() = car;

    route_paths_.clear();

    to_clear_ = false;
  }

  AdapterManager::Observe();
  UpdateWithLatestObserved("Chassis", AdapterManager::GetChassis());
  UpdateWithLatestObserved("Gps", AdapterManager::GetGps());
  UpdateWithLatestObserved("Localization", AdapterManager::GetLocalization());

  // Clear objects received from last frame and populate with the new objects.
  // TODO(siyangy, unacao): For now we are assembling the simulation_world with
  // latest received perception, prediction and planning message. However, they
  // may not always be perfectly aligned and belong to the same frame.
  obj_map_.clear();
  world_.clear_object();
  UpdateWithLatestObserved("PerceptionObstacles",
                           AdapterManager::GetPerceptionObstacles());
  UpdateWithLatestObserved("PerceptionTrafficLight",
                           AdapterManager::GetTrafficLightDetection(), false);
  UpdateWithLatestObserved("PredictionObstacles",
                           AdapterManager::GetPrediction());
  UpdateWithLatestObserved("Planning", AdapterManager::GetPlanning());
  UpdateWithLatestObserved("ControlCommand",
                           AdapterManager::GetControlCommand());
  UpdateWithLatestObserved("Navigation", AdapterManager::GetNavigation());
  UpdateWithLatestObserved("RelativeMap", AdapterManager::GetRelativeMap());

  for (const auto &kv : obj_map_) {
    *world_.add_object() = kv.second;
  }

  UpdateDelays();

  world_.set_sequence_num(world_.sequence_num() + 1);
  world_.set_timestamp(apollo::common::time::AsInt64<millis>(Clock::Now()));
}

void SimulationWorldService::UpdateDelays() {
  auto *delays = world_.mutable_delay();
  delays->set_chassis(SecToMs(AdapterManager::GetChassis()->GetDelaySec()));
  delays->set_localization(
      SecToMs(AdapterManager::GetLocalization()->GetDelaySec()));
  delays->set_perception_obstacle(
      SecToMs(AdapterManager::GetPerceptionObstacles()->GetDelaySec()));
  delays->set_planning(SecToMs(AdapterManager::GetPlanning()->GetDelaySec()));
  delays->set_prediction(
      SecToMs(AdapterManager::GetPrediction()->GetDelaySec()));
  delays->set_traffic_light(
      SecToMs(AdapterManager::GetTrafficLightDetection()->GetDelaySec()));
  delays->set_control(
      SecToMs(AdapterManager::GetControlCommand()->GetDelaySec()));
}

void SimulationWorldService::GetWireFormatString(
    double radius, std::string *sim_world,
    std::string *sim_world_with_planning_data) {
  PopulateMapInfo(radius);

  world_.SerializeToString(sim_world_with_planning_data);

  world_.clear_planning_data();
  world_.SerializeToString(sim_world);
}

Json SimulationWorldService::GetUpdateAsJson(double radius) const {
  std::string sim_world_json_string;
  MessageToJsonString(world_, &sim_world_json_string);

  Json update;
  update["type"] = "SimWorldUpdate";
  update["timestamp"] = apollo::common::time::AsInt64<millis>(Clock::Now());
  update["world"] = sim_world_json_string;

  return update;
}

void SimulationWorldService::GetMapElementIds(double radius,
                                              MapElementIds *ids) const {
  // Gather required map element ids based on current location.
  apollo::common::PointENU point;
  const auto &adc = world_.auto_driving_car();
  point.set_x(adc.position_x() + map_service_->GetXOffset());
  point.set_y(adc.position_y() + map_service_->GetYOffset());
  map_service_->CollectMapElementIds(point, radius, ids);
}

void SimulationWorldService::PopulateMapInfo(double radius) {
  world_.clear_map_element_ids();
  GetMapElementIds(radius, world_.mutable_map_element_ids());
  world_.set_map_hash(map_service_->CalculateMapHash(world_.map_element_ids()));
  world_.set_map_radius(radius);
}

const Map &SimulationWorldService::GetRelativeMap() const {
  return relative_map_;
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const MonitorMessage &monitor_msg) {
  std::vector<MonitorMessageItem> updated;
  updated.reserve(SimulationWorldService::kMaxMonitorItems);
  // Save the latest messages at the top of the history.
  const int updated_size = std::min(monitor_msg.item_size(),
                                    SimulationWorldService::kMaxMonitorItems);
  std::copy(monitor_msg.item().begin(),
            monitor_msg.item().begin() + updated_size,
            std::back_inserter(updated));

  // Copy over the previous messages until there is no more history or
  // the max number of total messages has been hit.
  auto history = world_.monitor().item();
  const int history_size = std::min(
      history.size(), SimulationWorldService::kMaxMonitorItems - updated_size);
  if (history_size > 0) {
    std::copy(history.begin(), history.begin() + history_size,
              std::back_inserter(updated));
  }

  // Refresh the monitor message list in simulation_world.
  *world_.mutable_monitor()->mutable_item() = {updated.begin(), updated.end()};
  world_.mutable_monitor()->mutable_header()->set_timestamp_sec(
      Clock::NowInSeconds());
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const LocalizationEstimate &localization) {
  Object *auto_driving_car = world_.mutable_auto_driving_car();
  const auto &pose = localization.pose();

  // Updates position with the input localization message.
  auto_driving_car->set_position_x(pose.position().x() +
                                   map_service_->GetXOffset());
  auto_driving_car->set_position_y(pose.position().y() +
                                   map_service_->GetYOffset());
  auto_driving_car->set_heading(pose.heading());

  // Updates acceleration with the input localization message.
  auto_driving_car->set_speed_acceleration(CalculateAcceleration(
      pose.linear_acceleration(), pose.linear_velocity()));

  // Updates the timestamp with the timestamp inside the localization
  // message header. It is done on both the SimulationWorld object
  // itself and its auto_driving_car() field.
  auto_driving_car->set_timestamp_sec(localization.header().timestamp_sec());

  ready_to_push_.store(true);
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Gps &gps) {
  Object *gps_position = world_.mutable_gps();
  gps_position->set_timestamp_sec(gps.header().timestamp_sec());

  const auto &pose = gps.localization();
  gps_position->set_position_x(pose.position().x() +
                               map_service_->GetXOffset());
  gps_position->set_position_y(pose.position().y() +
                               map_service_->GetYOffset());

  double heading = apollo::common::math::QuaternionToHeading(
      pose.orientation().qw(), pose.orientation().qx(), pose.orientation().qy(),
      pose.orientation().qz());
  gps_position->set_heading(heading);
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Chassis &chassis) {
  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  Object *auto_driving_car = world_.mutable_auto_driving_car();

  auto_driving_car->set_speed(chassis.speed_mps());
  auto_driving_car->set_throttle_percentage(chassis.throttle_percentage());
  auto_driving_car->set_brake_percentage(chassis.brake_percentage());

  // In case of out-of-range percentages, reduces it to zero.
  double angle_percentage = chassis.steering_percentage();
  if (angle_percentage > 100 || angle_percentage < -100) {
    angle_percentage = 0;
  }
  auto_driving_car->set_steering_percentage(angle_percentage);

  double steering_angle =
      angle_percentage / 100.0 * vehicle_param.max_steer_angle();
  auto_driving_car->set_steering_angle(steering_angle);

  double kappa = std::tan(steering_angle / vehicle_param.steer_ratio()) /
                 vehicle_param.length();
  auto_driving_car->set_kappa(kappa);

  UpdateTurnSignal(chassis.signal(), auto_driving_car);

  auto_driving_car->set_disengage_type(DeduceDisengageType(chassis));
}

Object &SimulationWorldService::CreateWorldObjectIfAbsent(
    const PerceptionObstacle &obstacle) {
  const std::string id = std::to_string(obstacle.id());
  // Create a new world object and put it into object map if the id does not
  // exist in the map yet.
  if (!apollo::common::util::ContainsKey(obj_map_, id)) {
    Object &world_obj = obj_map_[id];
    SetObstacleInfo(obstacle, &world_obj);
    SetObstaclePolygon(obstacle, &world_obj);
    SetObstacleType(obstacle, &world_obj);
  }
  return obj_map_[id];
}

void SimulationWorldService::SetObstacleInfo(const PerceptionObstacle &obstacle,
                                             Object *world_object) {
  if (world_object == nullptr) {
    return;
  }

  world_object->set_id(std::to_string(obstacle.id()));
  world_object->set_position_x(obstacle.position().x() +
                               map_service_->GetXOffset());
  world_object->set_position_y(obstacle.position().y() +
                               map_service_->GetYOffset());
  world_object->set_heading(obstacle.theta());
  world_object->set_length(obstacle.length());
  world_object->set_width(obstacle.width());
  world_object->set_height(obstacle.height());
  world_object->set_speed(
      std::hypot(obstacle.velocity().x(), obstacle.velocity().y()));
  world_object->set_speed_heading(
      std::atan2(obstacle.velocity().y(), obstacle.velocity().x()));
  world_object->set_timestamp_sec(obstacle.timestamp());
  world_object->set_confidence(obstacle.confidence());
}

void SimulationWorldService::SetObstaclePolygon(
    const PerceptionObstacle &obstacle, Object *world_object) {
  if (world_object == nullptr) {
    return;
  }

  using apollo::common::util::PairHash;
  std::unordered_set<std::pair<double, double>, PairHash> seen_points;
  world_object->clear_polygon_point();
  for (const auto &point : obstacle.polygon_point()) {
    // Filter out duplicate xy pairs.
    std::pair<double, double> xy_pair = {point.x(), point.y()};
    if (seen_points.count(xy_pair) == 0) {
      PolygonPoint *poly_pt = world_object->add_polygon_point();
      poly_pt->set_x(point.x() + map_service_->GetXOffset());
      poly_pt->set_y(point.y() + map_service_->GetYOffset());
      seen_points.insert(xy_pair);
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const PerceptionObstacles &obstacles) {
  for (const auto &obstacle : obstacles.perception_obstacle()) {
    auto &world_obj = CreateWorldObjectIfAbsent(obstacle);
    if (obstacles.has_cipv_info() &&
        (obstacles.cipv_info().cipv_id() == obstacle.id())) {
      world_obj.set_type(Object_Type_CIPV);
    }
  }

  if (obstacles.has_lane_marker()) {
    world_.mutable_lane_marker()->CopyFrom(obstacles.lane_marker());
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const TrafficLightDetection &traffic_light_detection) {
  Object *signal = world_.mutable_traffic_signal();
  if (traffic_light_detection.traffic_light_size() > 0) {
    const auto &traffic_light = traffic_light_detection.traffic_light(0);
    signal->set_current_signal(
        apollo::perception::TrafficLight_Color_Name(traffic_light.color()));
  } else {
    signal->set_current_signal("");
  }
}

void SimulationWorldService::UpdatePlanningTrajectory(
    const ADCTrajectory &trajectory) {
  const double cutoff_time = world_.auto_driving_car().timestamp_sec();
  const double header_time = trajectory.header().timestamp_sec();

  world_.set_planning_time(header_time);

  // Collect trajectory
  util::TrajectoryPointCollector collector(&world_);

  bool collecting_started = false;
  for (const TrajectoryPoint &point : trajectory.trajectory_point()) {
    // Trajectory points with a timestamp older than the cutoff time
    // (which is effectively the timestamp of the most up-to-date
    // localization/chassis message) will be dropped.
    if (collecting_started ||
        point.relative_time() + header_time >= cutoff_time) {
      collecting_started = true;
      collector.Collect(point, header_time);
    }
  }
  for (int i = 0; i < world_.planning_trajectory_size(); ++i) {
    auto traj_pt = world_.mutable_planning_trajectory(i);
    traj_pt->set_position_x(traj_pt->position_x() + map_service_->GetXOffset());
    traj_pt->set_position_y(traj_pt->position_y() + map_service_->GetYOffset());
  }

  // Update engage advice.
  // This is a temporary solution, the advice will come from monitor later
  if (trajectory.has_engage_advice()) {
    world_.set_engage_advice(
        EngageAdvice_Advice_Name(trajectory.engage_advice().advice()));
  }
}

void SimulationWorldService::UpdateMainStopDecision(
    const apollo::planning::MainDecision &main_decision,
    double update_timestamp_sec, Object *world_main_decision) {
  apollo::common::math::Vec2d stop_pt;
  double stop_heading = 0.0;
  auto decision = world_main_decision->add_decision();
  decision->set_type(Decision::STOP);
  if (main_decision.has_not_ready()) {
    // The car is not ready!
    // Use the current ADC pose since it is better not to self-drive.
    stop_pt.set_x(world_.auto_driving_car().position_x());
    stop_pt.set_y(world_.auto_driving_car().position_y());
    stop_heading = world_.auto_driving_car().heading();
    decision->set_stopreason(Decision::STOP_REASON_NOT_READY);
  } else if (main_decision.has_estop()) {
    // Emergency stop.
    // Use the current ADC pose since it is better to stop immediately.
    stop_pt.set_x(world_.auto_driving_car().position_x());
    stop_pt.set_y(world_.auto_driving_car().position_y());
    stop_heading = world_.auto_driving_car().heading();
    decision->set_stopreason(Decision::STOP_REASON_EMERGENCY);
    world_.mutable_auto_driving_car()->set_current_signal("EMERGENCY");
  } else {
    // Normal stop.
    const apollo::planning::MainStop &stop = main_decision.stop();
    stop_pt.set_x(stop.stop_point().x());
    stop_pt.set_y(stop.stop_point().y());
    stop_heading = stop.stop_heading();
    if (stop.has_reason_code()) {
      SetStopReason(stop.reason_code(), decision);
    }
  }

  decision->set_position_x(stop_pt.x() + map_service_->GetXOffset());
  decision->set_position_y(stop_pt.y() + map_service_->GetYOffset());
  decision->set_heading(stop_heading);

  world_main_decision->set_position_x(decision->position_x());
  world_main_decision->set_position_y(decision->position_y());
  world_main_decision->set_heading(decision->heading());
  world_main_decision->set_timestamp_sec(update_timestamp_sec);
}

template <typename MainDecision>
void SimulationWorldService::UpdateMainChangeLaneDecision(
    const MainDecision &decision, Object *world_main_decision) {
  if (decision.has_change_lane_type() &&
      (decision.change_lane_type() == apollo::routing::ChangeLaneType::LEFT ||
       decision.change_lane_type() == apollo::routing::ChangeLaneType::RIGHT)) {
    auto *change_lane_decision = world_main_decision->add_decision();
    change_lane_decision->set_change_lane_type(decision.change_lane_type());
    change_lane_decision->set_position_x(
        world_.auto_driving_car().position_x() + map_service_->GetXOffset());
    change_lane_decision->set_position_y(
        world_.auto_driving_car().position_y() + map_service_->GetYOffset());
    change_lane_decision->set_heading(world_.auto_driving_car().heading());
  }
}

bool SimulationWorldService::LocateMarker(
    const apollo::planning::ObjectDecisionType &decision,
    Decision *world_decision) {
  apollo::common::PointENU fence_point;
  double heading;
  if (decision.has_stop() && decision.stop().has_stop_point()) {
    world_decision->set_type(Decision_Type_STOP);
    fence_point = decision.stop().stop_point();
    heading = decision.stop().stop_heading();
  } else if (decision.has_follow() && decision.follow().has_fence_point()) {
    world_decision->set_type(Decision_Type_FOLLOW);
    fence_point = decision.follow().fence_point();
    heading = decision.follow().fence_heading();
  } else if (decision.has_yield() && decision.yield().has_fence_point()) {
    world_decision->set_type(Decision_Type_YIELD);
    fence_point = decision.yield().fence_point();
    heading = decision.yield().fence_heading();
  } else if (decision.has_overtake() && decision.overtake().has_fence_point()) {
    world_decision->set_type(Decision_Type_OVERTAKE);
    fence_point = decision.overtake().fence_point();
    heading = decision.overtake().fence_heading();
  } else {
    return false;
  }

  world_decision->set_position_x(fence_point.x() + map_service_->GetXOffset());
  world_decision->set_position_y(fence_point.y() + map_service_->GetYOffset());
  world_decision->set_heading(heading);
  return true;
}

void SimulationWorldService::FindNudgeRegion(
    const apollo::planning::ObjectDecisionType &decision,
    const Object &world_obj, Decision *world_decision) {
  std::vector<apollo::common::math::Vec2d> points;
  for (auto &polygon_pt : world_obj.polygon_point()) {
    points.emplace_back(polygon_pt.x() + map_service_->GetXOffset(),
                        polygon_pt.y() + map_service_->GetYOffset());
  }
  const apollo::common::math::Polygon2d obj_polygon(points);
  const apollo::common::math::Polygon2d &nudge_polygon =
      obj_polygon.ExpandByDistance(std::fabs(decision.nudge().distance_l()));
  const std::vector<apollo::common::math::Vec2d> &nudge_points =
      nudge_polygon.points();
  for (auto &nudge_pt : nudge_points) {
    PolygonPoint *poly_pt = world_decision->add_polygon_point();
    poly_pt->set_x(nudge_pt.x());
    poly_pt->set_y(nudge_pt.y());
  }
  world_decision->set_type(Decision_Type_NUDGE);
}

void SimulationWorldService::UpdateDecision(const DecisionResult &decision_res,
                                            double header_time) {
  // Update turn signal.
  UpdateTurnSignal(decision_res.vehicle_signal(),
                   world_.mutable_auto_driving_car());

  const auto &main_decision = decision_res.main_decision();

  // Update speed limit.
  if (main_decision.target_lane_size() > 0) {
    world_.set_speed_limit(main_decision.target_lane(0).speed_limit());
  }

  // Update relevant main stop with reason and change lane.
  world_.clear_main_decision();
  Object *world_main_decision = world_.mutable_main_decision();
  if (main_decision.has_not_ready() || main_decision.has_estop() ||
      main_decision.has_stop()) {
    UpdateMainStopDecision(main_decision, header_time, world_main_decision);
  }
  if (main_decision.has_stop()) {
    UpdateMainChangeLaneDecision(main_decision.stop(), world_main_decision);
  } else if (main_decision.has_cruise()) {
    UpdateMainChangeLaneDecision(main_decision.cruise(), world_main_decision);
  }

  // Update obstacle decision.
  for (const auto &obj_decision : decision_res.object_decision().decision()) {
    if (obj_decision.has_perception_id()) {
      int id = obj_decision.perception_id();
      Object &world_obj = obj_map_[std::to_string(id)];
      if (!world_obj.has_type()) {
        world_obj.set_type(Object_Type_VIRTUAL);
        ADEBUG << id << " is not a current perception object";
      }

      for (const auto &decision : obj_decision.object_decision()) {
        Decision *world_decision = world_obj.add_decision();
        world_decision->set_type(Decision_Type_IGNORE);
        if (decision.has_stop() || decision.has_follow() ||
            decision.has_yield() || decision.has_overtake()) {
          if (!LocateMarker(decision, world_decision)) {
            AWARN << "No decision marker position found for object id=" << id;
            continue;
          }
        } else if (decision.has_nudge()) {
          if (world_obj.polygon_point_size() == 0) {
            if (world_obj.type() == Object_Type_VIRTUAL) {
              AWARN << "No current perception object with id=" << id
                    << " for nudge decision";
            } else {
              AWARN << "No polygon points found for object id=" << id;
            }
            continue;
          }
          FindNudgeRegion(decision, world_obj, world_decision);
        } else if (decision.has_sidepass()) {
          world_decision->set_type(Decision_Type_SIDEPASS);
        }
      }

      world_obj.set_timestamp_sec(
          std::max(world_obj.timestamp_sec(), header_time));
    }
  }
}

void SimulationWorldService::DownsamplePath(const common::Path &path,
                                            common::Path *downsampled_path) {
  auto sampled_indices = DownsampleByAngle(path.path_point(), kAngleThreshold);

  downsampled_path->set_name(path.name());
  for (int index : sampled_indices) {
    const auto &path_point = path.path_point()[index];
    auto *point = downsampled_path->add_path_point();
    point->CopyFrom(path_point);
  }
}

void SimulationWorldService::UpdatePlanningData(const PlanningData &data) {
  auto *planning_data = world_.mutable_planning_data();

  size_t max_interval = 10;

  // Update SL Frame
  planning_data->mutable_sl_frame()->CopyFrom(data.sl_frame());

  // Update DP path
  planning_data->mutable_dp_poly_graph()->CopyFrom(data.dp_poly_graph());

  // Update ST Graph
  planning_data->clear_st_graph();
  for (auto &graph : data.st_graph()) {
    auto *st_graph = planning_data->add_st_graph();
    st_graph->set_name(graph.name());
    st_graph->mutable_boundary()->CopyFrom(graph.boundary());
    if (graph.has_kernel_cruise_ref()) {
      st_graph->mutable_kernel_cruise_ref()->CopyFrom(
          graph.kernel_cruise_ref());
    }
    if (graph.has_kernel_follow_ref()) {
      st_graph->mutable_kernel_follow_ref()->CopyFrom(
          graph.kernel_follow_ref());
    }
    if (graph.has_speed_constraint()) {
      st_graph->mutable_speed_constraint()->CopyFrom(graph.speed_constraint());
    }

    // downsample speed_profile and speed_limit
    // The x-axis range is always [-10, 200], downsample to ~200 points but skip
    // max 10 points
    size_t profile_downsample_interval =
        std::max(1, (graph.speed_profile_size() / 200));
    profile_downsample_interval =
        std::min(profile_downsample_interval, max_interval);
    DownsampleSpeedPointsByInterval(graph.speed_profile(),
                                    profile_downsample_interval,
                                    st_graph->mutable_speed_profile());

    size_t limit_downsample_interval =
        std::max(1, (graph.speed_limit_size() / 200));
    limit_downsample_interval =
        std::min(limit_downsample_interval, max_interval);
    DownsampleSpeedPointsByInterval(graph.speed_limit(),
                                    limit_downsample_interval,
                                    st_graph->mutable_speed_limit());
  }

  // Update Speed Plan
  planning_data->clear_speed_plan();
  for (auto &plan : data.speed_plan()) {
    if (plan.speed_point_size() > 0) {
      auto *downsampled_plan = planning_data->add_speed_plan();
      downsampled_plan->set_name(plan.name());

      // Downsample the speed plan for frontend display.
      // The x-axis range is always [-2, 10], downsample to ~80 points
      size_t interval = std::max(1, (plan.speed_point_size() / 80));
      interval = std::min(interval, max_interval);
      DownsampleSpeedPointsByInterval(plan.speed_point(), interval,
                                      downsampled_plan->mutable_speed_point());
    }
  }

  // Update path
  planning_data->clear_path();
  for (auto &path : data.path()) {
    DownsamplePath(path, planning_data->add_path());
  }
}

template <typename Points>
void SimulationWorldService::DownsampleSpeedPointsByInterval(
    const Points &points, size_t downsampleInterval,
    Points *downsampled_points) {
  if (points.size() == 0) {
    return;
  }

  for (int i = 0; i < points.size() - 1; i += downsampleInterval) {
    auto *point = downsampled_points->Add();
    point->set_s(points[i].s());
    point->set_t(points[i].t());
    point->set_v(points[i].v());
  }

  // add the last point
  auto *point = downsampled_points->Add();
  point->set_s(points[points.size() - 1].s());
  point->set_t(points[points.size() - 1].t());
  point->set_v(points[points.size() - 1].v());
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const ADCTrajectory &trajectory) {
  const double header_time = trajectory.header().timestamp_sec();

  UpdatePlanningTrajectory(trajectory);

  UpdateDecision(trajectory.decision(), header_time);

  UpdatePlanningData(trajectory.debug().planning_data());

  world_.mutable_latency()->set_planning(
      trajectory.latency_stats().total_time_ms());
}

void SimulationWorldService::CreatePredictionTrajectory(
    const PredictionObstacle &obstacle, Object *world_object) {
  for (const auto &traj : obstacle.trajectory()) {
    Prediction *prediction = world_object->add_prediction();
    prediction->set_probability(traj.probability());

    std::vector<PathPoint> points;
    for (const auto &point : traj.trajectory_point()) {
      points.push_back(point.path_point());
    }
    auto sampled_indices = DownsampleByAngle(points, kAngleThreshold);

    for (auto index : sampled_indices) {
      const auto &point = points[index];
      PolygonPoint *world_point = prediction->add_predicted_trajectory();
      world_point->set_x(point.x() + map_service_->GetXOffset());
      world_point->set_y(point.y() + map_service_->GetYOffset());
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const PredictionObstacles &obstacles) {
  for (const auto &obstacle : obstacles.prediction_obstacle()) {
    // Note: There's a perfect one-to-one mapping between the perception
    // obstacles and prediction obstacles within the same frame. Creating a new
    // world object here is only possible when we happen to be processing a
    // percpetion and prediction message from two frames.
    auto &world_obj = CreateWorldObjectIfAbsent(obstacle.perception_obstacle());

    // Add prediction trajectory to the object.
    CreatePredictionTrajectory(obstacle, &world_obj);

    world_obj.set_timestamp_sec(
        std::max(obstacle.timestamp(), world_obj.timestamp_sec()));
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const RoutingResponse &routing_response) {
  std::vector<Path> paths;
  if (!map_service_->GetPathsFromRouting(routing_response, &paths)) {
    return;
  }

  world_.clear_route_path();
  route_paths_.clear();
  world_.set_routing_time(routing_response.header().timestamp_sec());

  for (const Path &path : paths) {
    // Downsample the path points for frontend display.
    auto sampled_indices =
        DownsampleByAngle(path.path_points(), kAngleThreshold);

    route_paths_.emplace_back();
    RoutePath *route_path = &route_paths_.back();
    for (int index : sampled_indices) {
      const auto &path_point = path.path_points()[index];
      PolygonPoint *route_point = route_path->add_point();
      route_point->set_x(path_point.x() + map_service_->GetXOffset());
      route_point->set_y(path_point.y() + map_service_->GetYOffset());
    }

    // Populate route path
    if (FLAGS_sim_world_with_routing_path) {
      auto *new_path = world_.add_route_path();
      *new_path = *route_path;
    }
  }
}

Json SimulationWorldService::GetRoutePathAsJson() const {
  Json response;
  response["routingTime"] = world_.routing_time();
  response["routePath"] = Json::array();
  for (const auto &route_path : route_paths_) {
    Json path;
    path["point"] = Json::array();
    for (const auto &route_point : route_path.point()) {
      path["point"].push_back({{"x", route_point.x()},
                               {"y", route_point.y()},
                               {"z", route_point.z()}});
    }
    response["routePath"].push_back(path);
  }
  return response;
}

void SimulationWorldService::ReadRoutingFromFile(
    const std::string &routing_response_file) {
  RoutingResponse routing_response;
  if (!GetProtoFromFile(routing_response_file, &routing_response)) {
    AWARN << "Unable to read routing response from file: "
          << routing_response_file;
    return;
  }
  AINFO << "Loaded routing from " << routing_response_file;

  sleep(1);  // Wait to make sure the connection has been established before
             // publishing.
  AdapterManager::PublishRoutingResponse(routing_response);
  AINFO << "Published RoutingResponse read from file.";
}

void SimulationWorldService::RegisterMessageCallbacks() {
  AdapterManager::AddMonitorCallback(
      &SimulationWorldService::UpdateSimulationWorld, this);
  AdapterManager::AddRoutingResponseCallback(
      &SimulationWorldService::UpdateSimulationWorld, this);
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const ControlCommand &control_command) {
  auto *control_data = world_.mutable_control_data();
  control_data->set_timestamp_sec(control_command.header().timestamp_sec());

  if (control_command.has_debug()) {
    auto &debug = control_command.debug();
    if (debug.has_simple_lon_debug() && debug.has_simple_lat_debug()) {
      auto &simple_lon = debug.simple_lon_debug();
      if (simple_lon.has_station_error()) {
        control_data->set_station_error(simple_lon.station_error());
      }
      auto &simple_lat = debug.simple_lat_debug();
      if (simple_lat.has_heading_error()) {
        control_data->set_heading_error(simple_lat.heading_error());
      }
      if (simple_lat.has_lateral_error()) {
        control_data->set_lateral_error(simple_lat.lateral_error());
      }
    } else if (debug.has_simple_mpc_debug()) {
      auto &simple_mpc = debug.simple_mpc_debug();
      if (simple_mpc.has_station_error()) {
        control_data->set_station_error(simple_mpc.station_error());
      }
      if (simple_mpc.has_heading_error()) {
        control_data->set_heading_error(simple_mpc.heading_error());
      }
      if (simple_mpc.has_lateral_error()) {
        control_data->set_lateral_error(simple_mpc.lateral_error());
      }
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const NavigationInfo &navigation_info) {
  world_.clear_navigation_path();
  for (auto &navigation_path : navigation_info.navigation_path()) {
    if (navigation_path.has_path()) {
      DownsamplePath(navigation_path.path(), world_.add_navigation_path());
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const MapMsg &map_msg) {
  if (map_msg.has_hdmap()) {
    relative_map_.CopyFrom(map_msg.hdmap());
    for (int i = 0; i < relative_map_.lane_size(); ++i) {
      auto *lane = relative_map_.mutable_lane(i);
      lane->clear_left_sample();
      lane->clear_right_sample();
      lane->clear_left_road_sample();
      lane->clear_right_road_sample();

      DownsampleCurve(lane->mutable_central_curve());
      DownsampleCurve(lane->mutable_left_boundary()->mutable_curve());
      DownsampleCurve(lane->mutable_right_boundary()->mutable_curve());
    }
  }
}

}  // namespace dreamview
}  // namespace apollo
