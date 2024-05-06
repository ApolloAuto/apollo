/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview_plus/backend/obstacle_updater/obstacle_updater.h"

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/dreamview_plus/proto/data_handler.pb.h"

#include "modules/common/util/map_util.h"
#include "modules/common/util/util.h"
namespace apollo {
namespace dreamview {

using apollo::common::util::PairHash;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
ObstacleUpdater::ObstacleUpdater(WebSocketHandler* websocket)
    : websocket_(websocket), node_(cyber::CreateNode("obstacle_updater")) {
  Init();
}

void ObstacleUpdater::Init() {
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate>& localization) {
        OnLocalization(localization);
      });
}
ObstacleChannelUpdater* ObstacleUpdater::GetObstacleChannelUpdater(
    const std::string& channel_name) {
  std::lock_guard<std::mutex> lck(channel_updater_map_mutex_);
  if (!obstacle_channel_updater_map_.count(channel_name)) {
    obstacle_channel_updater_map_[channel_name] =
        new ObstacleChannelUpdater(channel_name);
    obstacle_channel_updater_map_[channel_name]->perception_obstacle_reader_ =
        node_->CreateReader<PerceptionObstacles>(
            channel_name,
            [channel_name,
             this](const std::shared_ptr<PerceptionObstacles>& obstacles) {
              OnObstacles(obstacles, channel_name);
            });
  }
  return obstacle_channel_updater_map_[channel_name];
}
void ObstacleUpdater::StartStream(const double& time_interval_ms,
                                  const std::string& channel_name,
                                  nlohmann::json* subscribe_param) {
  if (channel_name.empty()) {
    AERROR << "Failed to subscribe channel for channel is empty";
    return;
  }
  if (time_interval_ms > 0) {
    ObstacleChannelUpdater* channel_updater =
        GetObstacleChannelUpdater(channel_name);
    if (channel_updater == nullptr) {
      AERROR << "Failed to subscribe channel: " << channel_name
             << "for channel updater not registered!";
      return;
    }
    channel_updater->timer_.reset(new cyber::Timer(
        time_interval_ms,
        [channel_name, this]() { this->OnTimer(channel_name); }, false));
    channel_updater->timer_->Start();
  } else {
    this->OnTimer(channel_name);
  }
}

void ObstacleUpdater::StopStream(const std::string& channel_name) {
  if (channel_name.empty()) {
    AERROR << "Failed to unsubscribe channel for channel is empty";
    return;
  }
  if (enabled_) {
    ObstacleChannelUpdater* channel_updater =
        GetObstacleChannelUpdater(channel_name);
    if (channel_updater->timer_) {
      channel_updater->timer_->Stop();
    }
    channel_updater->obj_map_.clear();
    channel_updater->obstacle_objects_.Clear();
    channel_updater->obstacles_.clear();
  }
}

void ObstacleUpdater::Stop() {
  if (enabled_) {
    obstacle_channel_updater_map_.clear();
  }
  enabled_ = false;
}

void ObstacleUpdater::OnTimer(const std::string& channel_name) {
  PublishMessage(channel_name);
}

void ObstacleUpdater::PublishMessage(const std::string& channel_name) {
  std::string to_send = "";
  GetObjects(&to_send, channel_name);
  StreamData stream_data;
  std::string stream_data_string;
  stream_data.set_action("stream");
  stream_data.set_data_name("obstacle");
  stream_data.set_channel_name(channel_name);
  std::vector<uint8_t> byte_data(to_send.begin(), to_send.end());
  stream_data.set_data(&(byte_data[0]), byte_data.size());
  stream_data.set_type("obstacle");
  stream_data.SerializeToString(&stream_data_string);
  websocket_->BroadcastBinaryData(stream_data_string);
}

void ObstacleUpdater::GetChannelMsg(std::vector<std::string>* channels) {
  enabled_ = true;
  GetChannelMsgWithFilter(channels, "perception.PerceptionObstacles", "");
}

void ObstacleUpdater::OnObstacles(
    const std::shared_ptr<PerceptionObstacles>& obstacles,
    const std::string& channel) {
  if (!enabled_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lck(updater_publish_mutex_);
    ObstacleChannelUpdater* channel_updater =
        GetObstacleChannelUpdater(channel);
    channel_updater->obstacles_.clear();
    for (auto& obstacle : obstacles->perception_obstacle()) {
      channel_updater->obstacles_.push_back(obstacle);
    }
  }
}

void ObstacleUpdater::OnLocalization(
    const std::shared_ptr<LocalizationEstimate>& localization) {
  if (!enabled_) {
    return;
  }
  adc_pose_ = localization->pose();
}
void ObstacleUpdater::GetObjects(std::string* to_send,
                                 std::string channel_name) {
  {
    std::lock_guard<std::mutex> lck(updater_publish_mutex_);
    ObstacleChannelUpdater* channel_updater =
        GetObstacleChannelUpdater(channel_name);

    if (channel_updater->obstacles_.empty()) {
      return;
    }
    channel_updater->obj_map_.clear();
    for (const auto& obstacle : channel_updater->obstacles_) {
      const std::string id = std::to_string(obstacle.id());
      if (!apollo::common::util::ContainsKey(channel_updater->obj_map_, id)) {
        Object& obj = channel_updater->obj_map_[id];
        SetObstacleInfo(obstacle, &obj);
        SetObstaclePolygon(obstacle, &obj);
        SetObstacleType(obstacle.type(), obstacle.sub_type(), &obj);
        SetObstacleSensorMeasurements(obstacle, &obj, channel_updater);
        SetObstacleSource(obstacle, &obj);
      } else {
        // The object is already in the map.
        // Only the coordinates of the polygon and its own coordinates are
        // updated.
        Object& obj = channel_updater->obj_map_[id];
        SetObstacleInfo(obstacle, &obj);
        SetObstaclePolygon(obstacle, &obj);
        SetObstacleSensorMeasurements(obstacle, &obj, channel_updater);
      }
    }
    Object auto_driving_car;
    SetADCPosition(&auto_driving_car);
    channel_updater->obstacle_objects_.Clear();
    for (const auto& kv : channel_updater->obj_map_) {
      *channel_updater->obstacle_objects_.add_obstacle() = kv.second;
    }
    channel_updater->obstacle_objects_.mutable_auto_driving_car()->CopyFrom(
        auto_driving_car);
    channel_updater->obstacle_objects_.SerializeToString(to_send);
  }
}
void ObstacleUpdater::SetObstacleInfo(const PerceptionObstacle& obstacle,
                                      Object* obj) {
  if (obj == nullptr) {
    return;
  }
  obj->set_id(std::to_string(obstacle.id()));
  obj->set_position_x(obstacle.position().x());
  obj->set_position_y(obstacle.position().y());
  obj->set_heading(obstacle.theta());
  obj->set_length(obstacle.length());
  obj->set_width(obstacle.width());
  obj->set_height(obstacle.height());
  obj->set_speed(std::hypot(obstacle.velocity().x(), obstacle.velocity().y()));
  obj->set_speed_heading(
      std::atan2(obstacle.velocity().y(), obstacle.velocity().x()));
  obj->set_timestamp_sec(obstacle.timestamp());
  obj->set_confidence(obstacle.has_confidence() ? obstacle.confidence() : 1);
}

void ObstacleUpdater::SetObstaclePolygon(const PerceptionObstacle& obstacle,
                                         Object* obj) {
  if (obj == nullptr) {
    return;
  }

  std::unordered_set<std::pair<double, double>, PairHash> seen_points;
  obj->clear_polygon_point();
  for (const auto& point : obstacle.polygon_point()) {
    std::pair<double, double> xy_pair = {point.x(), point.y()};
    if (seen_points.count(xy_pair) == 0) {
      PolygonPoint* poly_pt = obj->add_polygon_point();
      poly_pt->set_x(point.x());
      poly_pt->set_y(point.y());
      seen_points.insert(xy_pair);
    }
  }
}

void ObstacleUpdater::SetObstacleType(
    const PerceptionObstacle::Type obstacle_type,
    const PerceptionObstacle::SubType obstacle_subtype, Object* obj) {
  if (obj == nullptr) {
    return;
  }
  switch (obstacle_type) {
    case PerceptionObstacle::UNKNOWN:
      obj->set_type(Object_Type_UNKNOWN);
      break;
    case PerceptionObstacle::UNKNOWN_MOVABLE:
      obj->set_type(Object_Type_UNKNOWN_MOVABLE);
      break;
    case PerceptionObstacle::UNKNOWN_UNMOVABLE:
      obj->set_type(Object_Type_UNKNOWN_UNMOVABLE);
      break;
    case PerceptionObstacle::PEDESTRIAN:
      obj->set_type(Object_Type_PEDESTRIAN);
      break;
    case PerceptionObstacle::BICYCLE:
      obj->set_type(Object_Type_BICYCLE);
      break;
    case PerceptionObstacle::VEHICLE:
      obj->set_type(Object_Type_VEHICLE);
      break;
    default:
      obj->set_type(Object_Type_VIRTUAL);
  }

  obj->set_sub_type(obstacle_subtype);
}

void ObstacleUpdater::SetObstacleSource(const PerceptionObstacle& obstacle,
                                        Object* obj) {
  if (obj == nullptr || !obstacle.has_source()) {
    return;
  }
  const PerceptionObstacle::Source obstacle_source = obstacle.source();
  obj->set_source(obstacle_source);
  obj->clear_v2x_info();
  if (obstacle_source == PerceptionObstacle::V2X && obstacle.has_v2x_info()) {
    obj->mutable_v2x_info()->CopyFrom(obstacle.v2x_info());
  }
  return;
}

void ObstacleUpdater::SetObstacleSensorMeasurements(
    const PerceptionObstacle& obstacle, Object* obj,
    ObstacleChannelUpdater* channel_updater) {
  if (obj == nullptr) {
    return;
  }
  for (const auto& sensor : obstacle.measurements()) {
    Object* obj = (*(channel_updater->obstacle_objects_
                         .mutable_sensor_measurements()))[sensor.sensor_id()]
                      .add_sensor_measurement();
    obj->set_id(std::to_string(sensor.id()));
    obj->set_position_x(sensor.position().x());
    obj->set_position_y(sensor.position().y());
    obj->set_heading(sensor.theta());
    obj->set_length(sensor.length());
    obj->set_width(sensor.width());
    obj->set_height(sensor.height());
    SetObstacleType(sensor.type(), sensor.sub_type(), obj);
  }
}
void ObstacleUpdater::SetADCPosition(Object* auto_driving_car) {
  auto_driving_car->set_position_x(adc_pose_.position().x());
  auto_driving_car->set_position_y(adc_pose_.position().y());
  auto_driving_car->set_heading(adc_pose_.heading());
}

}  // namespace dreamview
}  // namespace apollo
