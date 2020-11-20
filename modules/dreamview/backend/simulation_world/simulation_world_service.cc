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

#include <unordered_set>

#include "absl/strings/str_split.h"
#include "google/protobuf/util/json_util.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/proto/vehicle_signal.pb.h"
#include "modules/dreamview/proto/simulation_world.pb.h"

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/points_downsampler.h"
#include "modules/common/util/util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::audio::AudioDetection;
using apollo::audio::AudioEvent;
using apollo::canbus::Chassis;
using apollo::common::DriveEvent;
using apollo::common::PathPoint;
using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::monitor::MonitorMessage;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::util::DownsampleByAngle;
using apollo::common::util::FillHeader;
using apollo::control::ControlCommand;
using apollo::cyber::Clock;
using apollo::cyber::common::GetProtoFromFile;
using apollo::hdmap::Curve;
using apollo::hdmap::Map;
using apollo::hdmap::Path;
using apollo::localization::Gps;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::perception::SensorMeasurement;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;
using apollo::perception::V2XInformation;
using apollo::planning::ADCTrajectory;
using apollo::planning::DecisionResult;
using apollo::planning::StopReasonCode;
using apollo::planning_internal::PlanningData;
using apollo::prediction::ObstaclePriority;
using apollo::prediction::PredictionObstacle;
using apollo::prediction::PredictionObstacles;
using apollo::relative_map::MapMsg;
using apollo::relative_map::NavigationInfo;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

using Json = nlohmann::json;
using ::google::protobuf::util::MessageToJsonString;

// Angle threshold is about 5.72 degree.
static constexpr double kAngleThreshold = 0.1;

namespace {

double CalculateAcceleration(
    const Point3D &acceleration, const Point3D &velocity,
    const apollo::canbus::Chassis_GearPosition &gear_location) {
  // Calculates the dot product of acceleration and velocity. The sign
  // of this projection indicates whether this is acceleration or
  // deceleration.
  double projection =
      acceleration.x() * velocity.x() + acceleration.y() * velocity.y();

  // Calculates the magnitude of the acceleration. Negate the value if
  // it is indeed a deceleration.
  double magnitude = std::hypot(acceleration.x(), acceleration.y());
  if (std::signbit(projection)) {
    magnitude = -magnitude;
  }

  // Negate the value if gear is reverse
  if (gear_location == Chassis::GEAR_REVERSE) {
    magnitude = -magnitude;
  }

  return magnitude;
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

void SetObstacleType(const PerceptionObstacle::Type obstacle_type,
                     const PerceptionObstacle::SubType obstacle_subtype,
                     Object *world_object) {
  if (world_object == nullptr) {
    return;
  }

  switch (obstacle_type) {
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

  world_object->set_sub_type(obstacle_subtype);
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
    case StopReasonCode::STOP_REASON_PULL_OVER:
      decision->set_stopreason(Decision::STOP_REASON_PULL_OVER);
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
  if (curve->segment().empty()) {
    return;
  }

  auto *line_segment = curve->mutable_segment(0)->mutable_line_segment();
  std::vector<PointENU> points(line_segment->point().begin(),
                               line_segment->point().end());
  line_segment->clear_point();

  // Downsample points by angle then by distance.
  std::vector<size_t> sampled_indices =
      DownsampleByAngle(points, kAngleThreshold);
  for (const size_t index : sampled_indices) {
    *line_segment->add_point() = points[index];
  }
}

inline double SecToMs(const double sec) { return sec * 1000.0; }

}  // namespace

constexpr int SimulationWorldService::kMaxMonitorItems;

SimulationWorldService::SimulationWorldService(const MapService *map_service,
                                               bool routing_from_file)
    : node_(cyber::CreateNode("simulation_world")),
      map_service_(map_service),
      monitor_logger_buffer_(MonitorMessageItem::SIMULATOR),
      ready_to_push_(false) {
  InitReaders();
  InitWriters();

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

void SimulationWorldService::InitReaders() {
  routing_request_reader_ =
      node_->CreateReader<RoutingRequest>(FLAGS_routing_request_topic);
  routing_response_reader_ =
      node_->CreateReader<RoutingResponse>(FLAGS_routing_response_topic);
  chassis_reader_ = node_->CreateReader<Chassis>(FLAGS_chassis_topic);
  gps_reader_ = node_->CreateReader<Gps>(FLAGS_gps_topic);
  localization_reader_ =
      node_->CreateReader<LocalizationEstimate>(FLAGS_localization_topic);
  perception_obstacle_reader_ =
      node_->CreateReader<PerceptionObstacles>(FLAGS_perception_obstacle_topic);
  perception_traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      FLAGS_traffic_light_detection_topic);
  prediction_obstacle_reader_ =
      node_->CreateReader<PredictionObstacles>(FLAGS_prediction_topic);
  planning_reader_ =
      node_->CreateReader<ADCTrajectory>(FLAGS_planning_trajectory_topic);
  control_command_reader_ =
      node_->CreateReader<ControlCommand>(FLAGS_control_command_topic);
  navigation_reader_ =
      node_->CreateReader<NavigationInfo>(FLAGS_navigation_topic);
  relative_map_reader_ = node_->CreateReader<MapMsg>(FLAGS_relative_map_topic);
  storytelling_reader_ = node_->CreateReader<Stories>(FLAGS_storytelling_topic);
  audio_detection_reader_ =
      node_->CreateReader<AudioDetection>(FLAGS_audio_detection_topic);

  audio_event_reader_ = node_->CreateReader<AudioEvent>(
      FLAGS_audio_event_topic,
      [this](const std::shared_ptr<AudioEvent> &audio_event) {
        this->PublishMonitorMessage(
            MonitorMessageItem::WARN,
            apollo::audio::AudioType_Name(audio_event->audio_type()));
      });
  drive_event_reader_ = node_->CreateReader<DriveEvent>(
      FLAGS_drive_event_topic,
      [this](const std::shared_ptr<DriveEvent> &drive_event) {
        this->PublishMonitorMessage(MonitorMessageItem::WARN,
                                    drive_event->event());
      });
  cyber::ReaderConfig monitor_message_reader_config;
  monitor_message_reader_config.channel_name = FLAGS_monitor_topic;
  monitor_message_reader_config.pending_queue_size =
      FLAGS_monitor_msg_pending_queue_size;
  monitor_reader_ = node_->CreateReader<MonitorMessage>(
      monitor_message_reader_config,
      [this](const std::shared_ptr<MonitorMessage> &monitor_message) {
        std::unique_lock<std::mutex> lock(monitor_msgs_mutex_);
        monitor_msgs_.push_back(monitor_message);
      });
}

void SimulationWorldService::InitWriters() {
  navigation_writer_ =
      node_->CreateWriter<NavigationInfo>(FLAGS_navigation_topic);

  {  // configure QoS for routing request writer
    apollo::cyber::proto::RoleAttributes routing_request_attr;
    routing_request_attr.set_channel_name(FLAGS_routing_request_topic);
    auto qos = routing_request_attr.mutable_qos_profile();
    // only keeps the last message in history
    qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    // reliable transfer
    qos->set_reliability(
        apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    // when writer find new readers, send all its history messsage
    qos->set_durability(
        apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
    routing_request_writer_ =
        node_->CreateWriter<RoutingRequest>(routing_request_attr);
  }

  routing_response_writer_ =
      node_->CreateWriter<RoutingResponse>(FLAGS_routing_response_topic);
}

void SimulationWorldService::Update() {
  if (to_clear_) {
    // Clears received data.
    node_->ClearData();

    // Clears simulation world except for the car information.
    auto car = world_.auto_driving_car();
    world_.Clear();
    *world_.mutable_auto_driving_car() = car;

    {
      boost::unique_lock<boost::shared_mutex> writer_lock(route_paths_mutex_);
      route_paths_.clear();
    }

    to_clear_ = false;
  }

  node_->Observe();

  UpdateMonitorMessages();

  UpdateWithLatestObserved(routing_response_reader_.get(), false);
  UpdateWithLatestObserved(chassis_reader_.get());
  UpdateWithLatestObserved(gps_reader_.get());
  UpdateWithLatestObserved(localization_reader_.get());

  // Clear objects received from last frame and populate with the new objects.
  // TODO(siyangy, unacao): For now we are assembling the simulation_world with
  // latest received perception, prediction and planning message. However, they
  // may not always be perfectly aligned and belong to the same frame.
  obj_map_.clear();
  world_.clear_object();
  world_.clear_sensor_measurements();
  UpdateWithLatestObserved(audio_detection_reader_.get());
  UpdateWithLatestObserved(storytelling_reader_.get());
  UpdateWithLatestObserved(perception_obstacle_reader_.get());
  UpdateWithLatestObserved(perception_traffic_light_reader_.get(), false);
  UpdateWithLatestObserved(prediction_obstacle_reader_.get());
  UpdateWithLatestObserved(planning_reader_.get());
  UpdateWithLatestObserved(control_command_reader_.get());
  UpdateWithLatestObserved(navigation_reader_.get(), FLAGS_use_navigation_mode);
  UpdateWithLatestObserved(relative_map_reader_.get(),
                           FLAGS_use_navigation_mode);

  for (const auto &kv : obj_map_) {
    *world_.add_object() = kv.second;
  }

  UpdateDelays();
  UpdateLatencies();

  world_.set_sequence_num(world_.sequence_num() + 1);
  world_.set_timestamp(Clock::Now().ToSecond() * 1000);
}

void SimulationWorldService::UpdateDelays() {
  auto *delays = world_.mutable_delay();
  delays->set_chassis(SecToMs(chassis_reader_->GetDelaySec()));
  delays->set_localization(SecToMs(localization_reader_->GetDelaySec()));
  delays->set_perception_obstacle(
      SecToMs(perception_obstacle_reader_->GetDelaySec()));
  delays->set_planning(SecToMs(planning_reader_->GetDelaySec()));
  delays->set_prediction(SecToMs(prediction_obstacle_reader_->GetDelaySec()));
  delays->set_traffic_light(
      SecToMs(perception_traffic_light_reader_->GetDelaySec()));
  delays->set_control(SecToMs(control_command_reader_->GetDelaySec()));
}

void SimulationWorldService::UpdateLatencies() {
  UpdateLatency("chassis", chassis_reader_.get());
  UpdateLatency("localization", localization_reader_.get());
  UpdateLatency("perception", perception_obstacle_reader_.get());
  UpdateLatency("planning", planning_reader_.get());
  UpdateLatency("prediction", prediction_obstacle_reader_.get());
  UpdateLatency("control", control_command_reader_.get());
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
  update["timestamp"] = Clock::Now().ToSecond() * 1000;
  update["world"] = sim_world_json_string;

  return update;
}

void SimulationWorldService::GetMapElementIds(double radius,
                                              MapElementIds *ids) const {
  // Gather required map element ids based on current location.
  apollo::common::PointENU point;
  const auto &adc = world_.auto_driving_car();
  point.set_x(adc.position_x());
  point.set_y(adc.position_y());
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
      pose.linear_acceleration(), pose.linear_velocity(), gear_location_));

  // Updates the timestamp with the timestamp inside the localization
  // message header. It is done on both the SimulationWorld object
  // itself and its auto_driving_car() field.
  auto_driving_car->set_timestamp_sec(localization.header().timestamp_sec());
  ready_to_push_.store(true);
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Gps &gps) {
  if (gps.header().module_name() == "ShadowLocalization") {
    Object *shadow_localization_position = world_.mutable_shadow_localization();
    const auto &pose = gps.localization();
    shadow_localization_position->set_position_x(pose.position().x() +
                                                 map_service_->GetXOffset());
    shadow_localization_position->set_position_y(pose.position().y() +
                                                 map_service_->GetYOffset());
    shadow_localization_position->set_heading(pose.heading());
  } else {
    Object *gps_position = world_.mutable_gps();
    gps_position->set_timestamp_sec(gps.header().timestamp_sec());

    const auto &pose = gps.localization();
    gps_position->set_position_x(pose.position().x() +
                                 map_service_->GetXOffset());
    gps_position->set_position_y(pose.position().y() +
                                 map_service_->GetYOffset());

    double heading = apollo::common::math::QuaternionToHeading(
        pose.orientation().qw(), pose.orientation().qx(),
        pose.orientation().qy(), pose.orientation().qz());
    gps_position->set_heading(heading);
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Chassis &chassis) {
  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  Object *auto_driving_car = world_.mutable_auto_driving_car();

  double speed = chassis.speed_mps();
  gear_location_ = chassis.gear_location();
  if (gear_location_ == Chassis::GEAR_REVERSE) {
    speed = -speed;
  }
  auto_driving_car->set_speed(speed);
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
                 vehicle_param.wheel_base();
  auto_driving_car->set_kappa(kappa);

  UpdateTurnSignal(chassis.signal(), auto_driving_car);

  auto_driving_car->set_disengage_type(DeduceDisengageType(chassis));

  auto_driving_car->set_battery_percentage(
    chassis.battery_soc_percentage());
  auto_driving_car->set_gear_location(chassis.gear_location());
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Stories &stories) {
  world_.clear_stories();
  auto *world_stories = world_.mutable_stories();

  const google::protobuf::Descriptor *descriptor = stories.GetDescriptor();
  const google::protobuf::Reflection *reflection = stories.GetReflection();
  const int field_count = descriptor->field_count();
  for (int i = 0; i < field_count; ++i) {
    const google::protobuf::FieldDescriptor *field = descriptor->field(i);
    if (field->name() != "header") {
      (*world_stories)[field->name()] = reflection->HasField(stories, field);
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const AudioDetection &audio_detection) {
  world_.set_is_siren_on(audio_detection.is_siren());
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
    SetObstacleType(obstacle.type(), obstacle.sub_type(), &world_obj);
    SetObstacleSensorMeasurements(obstacle, &world_obj);
    SetObstacleSource(obstacle, &world_obj);
  }
  return obj_map_[id];
}

void SimulationWorldService::CreateWorldObjectFromSensorMeasurement(
    const SensorMeasurement &sensor, Object *world_object) {
  world_object->set_id(std::to_string(sensor.id()));
  world_object->set_position_x(sensor.position().x());
  world_object->set_position_y(sensor.position().y());
  world_object->set_heading(sensor.theta());
  world_object->set_length(sensor.length());
  world_object->set_width(sensor.width());
  world_object->set_height(sensor.height());
  SetObstacleType(sensor.type(), sensor.sub_type(), world_object);
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
  world_object->set_confidence(obstacle.has_confidence() ? obstacle.confidence()
                                                         : 1);
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

void SimulationWorldService::SetObstacleSensorMeasurements(
    const PerceptionObstacle &obstacle, Object *world_object) {
  if (world_object == nullptr) {
    return;
  }
  for (const auto &sensor : obstacle.measurements()) {
    Object *obj = (*(world_.mutable_sensor_measurements()))[sensor.sensor_id()]
                      .add_sensor_measurement();
    CreateWorldObjectFromSensorMeasurement(sensor, obj);
  }
}

void SimulationWorldService::SetObstacleSource(
    const apollo::perception::PerceptionObstacle &obstacle,
    Object *world_object) {
  if (world_object == nullptr || !obstacle.has_source()) {
    return;
  }
  const PerceptionObstacle::Source obstacle_source = obstacle.source();
  world_object->set_source(obstacle_source);
  world_object->clear_v2x_info();
  if (obstacle_source == PerceptionObstacle::V2X && obstacle.has_v2x_info()) {
    world_object->mutable_v2x_info()->CopyFrom(obstacle.v2x_info());
  }
  return;
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
  world_.clear_perceived_signal();
  for (const auto &traffic_light : traffic_light_detection.traffic_light()) {
    Object *signal = world_.add_perceived_signal();
    signal->set_id(traffic_light.id());
    signal->set_current_signal(TrafficLight_Color_Name(traffic_light.color()));
  }
}

void SimulationWorldService::UpdatePlanningTrajectory(
    const ADCTrajectory &trajectory) {
  // Collect trajectory
  world_.clear_planning_trajectory();
  const double base_time = trajectory.header().timestamp_sec();
  for (const TrajectoryPoint &point : trajectory.trajectory_point()) {
    Object *trajectory_point = world_.add_planning_trajectory();
    trajectory_point->set_timestamp_sec(point.relative_time() + base_time);
    trajectory_point->set_position_x(point.path_point().x() +
                                     map_service_->GetXOffset());
    trajectory_point->set_position_y(point.path_point().y() +
                                     map_service_->GetYOffset());
    trajectory_point->set_speed(point.v());
    trajectory_point->set_speed_acceleration(point.a());
    trajectory_point->set_kappa(point.path_point().kappa());
    trajectory_point->set_dkappa(point.path_point().dkappa());
    trajectory_point->set_heading(point.path_point().theta());
  }

  // Update engage advice.
  // This is a temporary solution, the advice will come from monitor later
  if (trajectory.has_engage_advice()) {
    world_.set_engage_advice(
        EngageAdvice_Advice_Name(trajectory.engage_advice().advice()));
  }
}

std::string formatDoubleToString(const double data) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << data;
  return ss.str();
}

void SimulationWorldService::UpdateRSSInfo(const ADCTrajectory &trajectory) {
  if (trajectory.has_rss_info()) {
    if (trajectory.rss_info().is_rss_safe()) {
      if (!world_.is_rss_safe()) {
        this->PublishMonitorMessage(MonitorMessageItem::INFO, "RSS safe.");
        world_.set_is_rss_safe(true);
      }
    } else {
      const double next_real_dist = trajectory.rss_info().cur_dist_lon();
      const double next_rss_safe_dist =
          trajectory.rss_info().rss_safe_dist_lon();
      // Not update RSS message if data keeps same.
      if (std::fabs(current_real_dist_ - next_real_dist) <
              common::math::kMathEpsilon &&
          std::fabs(current_rss_safe_dist_ - next_rss_safe_dist) <
              common::math::kMathEpsilon) {
        return;
      }
      this->PublishMonitorMessage(
          MonitorMessageItem::ERROR,
          "RSS unsafe: \ncurrent distance: " +
              formatDoubleToString(trajectory.rss_info().cur_dist_lon()) +
              "\nsafe distance: " +
              formatDoubleToString(trajectory.rss_info().rss_safe_dist_lon()));
      world_.set_is_rss_safe(false);
      current_real_dist_ = next_real_dist;
      current_rss_safe_dist_ = next_rss_safe_dist;
    }
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
    stop_pt.set_x(stop.stop_point().x() + map_service_->GetXOffset());
    stop_pt.set_y(stop.stop_point().y() + map_service_->GetYOffset());
    stop_heading = stop.stop_heading();
    if (stop.has_reason_code()) {
      SetStopReason(stop.reason_code(), decision);
    }
  }

  decision->set_position_x(stop_pt.x());
  decision->set_position_y(stop_pt.y());
  decision->set_heading(stop_heading);
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
    points.emplace_back(polygon_pt.x(), polygon_pt.y());
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
  if (world_main_decision->decision_size() > 0) {
    // set default position
    const auto &adc = world_.auto_driving_car();
    world_main_decision->set_position_x(adc.position_x());
    world_main_decision->set_position_y(adc.position_y());
    world_main_decision->set_heading(adc.heading());
    world_main_decision->set_timestamp_sec(header_time);
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
          if (decision.has_stop()) {
            // flag yielded obstacles
            for (auto obstacle_id : decision.stop().wait_for_obstacle()) {
              const std::vector<std::string> id_segments =
                  absl::StrSplit(obstacle_id, '_');
              if (id_segments.size() > 0) {
                obj_map_[id_segments[0]].set_yielded_obstacle(true);
              }
            }
          }
        } else if (decision.has_nudge()) {
          if (world_obj.polygon_point().empty()) {
            if (world_obj.type() == Object_Type_VIRTUAL) {
              AWARN << "No current perception object with id=" << id
                    << " for nudge decision";
            } else {
              AWARN << "No polygon points found for object id=" << id;
            }
            continue;
          }
          FindNudgeRegion(decision, world_obj, world_decision);
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
  for (const size_t index : sampled_indices) {
    *downsampled_path->add_path_point() =
        path.path_point(static_cast<int>(index));
  }
}

void SimulationWorldService::UpdatePlanningData(const PlanningData &data) {
  auto *planning_data = world_.mutable_planning_data();

  size_t max_interval = 10;

  // Update scenario
  if (data.has_scenario()) {
    planning_data->mutable_scenario()->CopyFrom(data.scenario());
  }

  // Update init point
  if (data.has_init_point()) {
    auto &planning_path_point = data.init_point().path_point();
    auto *world_obj_path_point =
        planning_data->mutable_init_point()->mutable_path_point();
    world_obj_path_point->set_x(planning_path_point.x() +
                                map_service_->GetXOffset());
    world_obj_path_point->set_y(planning_path_point.y() +
                                map_service_->GetYOffset());
    world_obj_path_point->set_theta(planning_path_point.theta());
  }

  // Update Chart
  planning_data->mutable_chart()->CopyFrom(data.chart());

  // Update SL Frame
  planning_data->mutable_sl_frame()->CopyFrom(data.sl_frame());

  // Update DP path
  if (data.has_dp_poly_graph()) {
    planning_data->mutable_dp_poly_graph()->CopyFrom(data.dp_poly_graph());
  }

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

  // Update pull over status
  planning_data->clear_pull_over();
  if (data.has_pull_over()) {
    planning_data->mutable_pull_over()->CopyFrom(data.pull_over());
  }

  // Update planning signal
  world_.clear_traffic_signal();
  if (data.has_signal_light() && data.signal_light().signal_size() > 0) {
    TrafficLight::Color current_signal = TrafficLight::UNKNOWN;
    int green_light_count = 0;

    for (auto &signal : data.signal_light().signal()) {
      switch (signal.color()) {
        case TrafficLight::RED:
        case TrafficLight::YELLOW:
        case TrafficLight::BLACK:
          current_signal = signal.color();
          break;
        case TrafficLight::GREEN:
          green_light_count++;
          break;
        default:
          break;
      }
    }

    if (green_light_count == data.signal_light().signal_size()) {
      current_signal = TrafficLight::GREEN;
    }

    world_.mutable_traffic_signal()->set_current_signal(
        TrafficLight_Color_Name(current_signal));
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const ADCTrajectory &trajectory) {
  const double header_time = trajectory.header().timestamp_sec();

  UpdatePlanningTrajectory(trajectory);

  UpdateRSSInfo(trajectory);

  UpdateDecision(trajectory.decision(), header_time);

  UpdatePlanningData(trajectory.debug().planning_data());

  Latency latency;
  latency.set_timestamp_sec(header_time);
  latency.set_total_time_ms(trajectory.latency_stats().total_time_ms());
  (*world_.mutable_latency())["planning"] = latency;
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

      const TrajectoryPoint &traj_point = traj.trajectory_point(index);
      if (traj_point.has_gaussian_info()) {
        const apollo::common::GaussianInfo &gaussian =
            traj_point.gaussian_info();

        auto *ellipse = world_point->mutable_gaussian_info();
        ellipse->set_ellipse_a(gaussian.ellipse_a());
        ellipse->set_ellipse_b(gaussian.ellipse_b());
        ellipse->set_theta_a(gaussian.theta_a());
      }
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
    // perception and prediction message from two frames.
    auto &world_obj = CreateWorldObjectIfAbsent(obstacle.perception_obstacle());

    // Add prediction trajectory to the object.
    CreatePredictionTrajectory(obstacle, &world_obj);

    // Add prediction priority
    if (obstacle.has_priority()) {
      world_obj.mutable_obstacle_priority()->CopyFrom(obstacle.priority());
    }

    world_obj.set_timestamp_sec(
        std::max(obstacle.timestamp(), world_obj.timestamp_sec()));
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const RoutingResponse &routing_response) {
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(route_paths_mutex_);
    if (world_.has_routing_time() &&
        world_.routing_time() == routing_response.header().timestamp_sec()) {
      // This routing response has been processed.
      return;
    }
  }

  std::vector<Path> paths;
  if (!map_service_->GetPathsFromRouting(routing_response, &paths)) {
    return;
  }

  world_.clear_route_path();

  std::vector<RoutePath> route_paths;
  for (const Path &path : paths) {
    // Downsample the path points for frontend display.
    auto sampled_indices =
        DownsampleByAngle(path.path_points(), kAngleThreshold);

    route_paths.emplace_back();
    RoutePath *route_path = &route_paths.back();
    for (const size_t index : sampled_indices) {
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
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(route_paths_mutex_);
    std::swap(route_paths, route_paths_);
    world_.set_routing_time(routing_response.header().timestamp_sec());
  }
}

Json SimulationWorldService::GetRoutePathAsJson() const {
  Json response;
  response["routePath"] = Json::array();
  std::vector<RoutePath> route_paths;
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(route_paths_mutex_);
    response["routingTime"] = world_.routing_time();
    route_paths = route_paths_;
  }
  for (const auto &route_path : route_paths) {
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
  auto routing_response = std::make_shared<RoutingResponse>();
  if (!GetProtoFromFile(routing_response_file, routing_response.get())) {
    AWARN << "Unable to read routing response from file: "
          << routing_response_file;
    return;
  }
  AINFO << "Loaded routing from " << routing_response_file;

  sleep(1);  // Wait to make sure the connection has been established before
             // publishing.
  routing_response_writer_->Write(routing_response);
  AINFO << "Published RoutingResponse read from file.";
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const ControlCommand &control_command) {
  auto *control_data = world_.mutable_control_data();
  const double header_time = control_command.header().timestamp_sec();
  control_data->set_timestamp_sec(header_time);

  Latency latency;
  latency.set_timestamp_sec(header_time);
  latency.set_total_time_ms(control_command.latency_stats().total_time_ms());
  (*world_.mutable_latency())["control"] = latency;

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
      if (simple_lat.has_current_target_point()) {
        control_data->mutable_current_target_point()->CopyFrom(
            simple_lat.current_target_point());
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

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const MonitorMessage &monitor_msg) {
  const int updated_size = std::min(monitor_msg.item_size(),
                                    SimulationWorldService::kMaxMonitorItems);
  // Save the latest messages at the end of the history.
  for (int idx = 0; idx < updated_size; ++idx) {
    auto *notification = world_.add_notification();
    notification->mutable_item()->CopyFrom(monitor_msg.item(idx));
    notification->set_timestamp_sec(monitor_msg.header().timestamp_sec());
  }

  int remove_size =
      world_.notification_size() - SimulationWorldService::kMaxMonitorItems;
  if (remove_size > 0) {
    auto *notifications = world_.mutable_notification();
    notifications->erase(notifications->begin(),
                         notifications->begin() + remove_size);
  }
}

void SimulationWorldService::UpdateMonitorMessages() {
  std::list<std::shared_ptr<MonitorMessage>> monitor_msgs;
  {
    std::unique_lock<std::mutex> lock(monitor_msgs_mutex_);
    monitor_msgs = monitor_msgs_;
    monitor_msgs_.clear();
  }

  for (const auto &monitor_msg : monitor_msgs) {
    UpdateSimulationWorld(*monitor_msg);
  }
}

void SimulationWorldService::DumpMessages() {
  DumpMessageFromReader(chassis_reader_.get());
  DumpMessageFromReader(prediction_obstacle_reader_.get());
  DumpMessageFromReader(routing_request_reader_.get());
  DumpMessageFromReader(routing_response_reader_.get());
  DumpMessageFromReader(localization_reader_.get());
  DumpMessageFromReader(planning_reader_.get());
  DumpMessageFromReader(control_command_reader_.get());
  DumpMessageFromReader(perception_obstacle_reader_.get());
  DumpMessageFromReader(perception_traffic_light_reader_.get());
  DumpMessageFromReader(relative_map_reader_.get());
  DumpMessageFromReader(navigation_reader_.get());
}

void SimulationWorldService::PublishNavigationInfo(
    const std::shared_ptr<NavigationInfo> &navigation_info) {
  FillHeader(FLAGS_dreamview_module_name, navigation_info.get());
  navigation_writer_->Write(navigation_info);
}

void SimulationWorldService::PublishRoutingRequest(
    const std::shared_ptr<RoutingRequest> &routing_request) {
  FillHeader(FLAGS_dreamview_module_name, routing_request.get());
  routing_request_writer_->Write(routing_request);
}

void SimulationWorldService::PublishMonitorMessage(
    apollo::common::monitor::MonitorMessageItem::LogLevel log_level,
    const std::string &msg) {
  monitor_logger_buffer_.AddMonitorMsgItem(log_level, msg);
  monitor_logger_buffer_.Publish();
}
}  // namespace dreamview
}  // namespace apollo
