/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/common/message_process.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/feature_output.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/math_util.h"

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::cyber::Clock;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::dreamview::HMIStatus;
using apollo::hdmap::ClearAreaInfoConstPtr;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::PNCJunctionInfoConstPtr;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::hdmap::StopSignInfoConstPtr;
using apollo::hdmap::YieldSignInfoConstPtr;
using apollo::localization::LocalizationEstimate;
using apollo::perception::TrafficLightDetection;
using apollo::prediction::PredictionObstacle;
using apollo::prediction::PredictionObstacles;
using apollo::routing::RoutingResponse;
using apollo::storytelling::CloseToJunction;
using apollo::storytelling::Stories;

bool MessageProcess::Init(const PlanningConfig& planning_config) {
  planning_config_.CopyFrom(planning_config);

  map_m_["Sunnyvale"] = "sunnyvale";
  map_m_["Sunnyvale Big Loop"] = "sunnyvale_big_loop";
  map_m_["Sunnyvale With Two Offices"] = "sunnyvale_with_two_offices";
  map_m_["Gomentum"] = "gomentum";
  map_m_["Sunnyvale Loop"] = "sunnyvale_loop";
  map_m_["San Mateo"] = "san_mateo";

  map_name_ = FLAGS_map_dir.substr(FLAGS_map_dir.find_last_of("/") + 1);

  obstacle_history_map_.clear();

  if (FLAGS_planning_offline_learning) {
    // offline process logging
    log_file_.open(FLAGS_planning_data_dir + "/learning_data.log",
                   std::ios_base::out | std::ios_base::app);
    start_time_ = std::chrono::system_clock::now();
    std::time_t now = std::time(nullptr);
    log_file_ << "UTC date and time: " << std::asctime(std::gmtime(&now))
              << "Local date and time: " << std::asctime(std::localtime(&now));
  }
  return true;
}

bool MessageProcess::Init(const PlanningConfig& planning_config,
                          const std::shared_ptr<DependencyInjector>& injector) {
  injector_ = injector;
  return Init(planning_config);
}

void MessageProcess::Close() {
  FeatureOutput::Clear();

  if (FLAGS_planning_offline_learning) {
    // offline process logging
    const std::string msg = absl::StrCat(
        "Total learning_data_frame number: ", total_learning_data_frame_num_);
    AINFO << msg;
    log_file_ << msg << std::endl;
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time_;
    log_file_ << "Time elapsed(sec): " << elapsed_seconds.count() << std::endl
              << std::endl;
    log_file_.close();
  }
}

void MessageProcess::OnChassis(const apollo::canbus::Chassis& chassis) {
  chassis_feature_.set_message_timestamp_sec(chassis.header().timestamp_sec());
  chassis_feature_.set_speed_mps(chassis.speed_mps());
  chassis_feature_.set_throttle_percentage(chassis.throttle_percentage());
  chassis_feature_.set_brake_percentage(chassis.brake_percentage());
  chassis_feature_.set_steering_percentage(chassis.steering_percentage());
  chassis_feature_.set_gear_location(chassis.gear_location());
}

void MessageProcess::OnHMIStatus(apollo::dreamview::HMIStatus hmi_status) {
  const std::string& current_map = hmi_status.current_map();
  if (map_m_.count(current_map) > 0) {
    map_name_ = map_m_[current_map];
    const std::string& map_base_folder = "/apollo/modules/map/data/";
    FLAGS_map_dir = map_base_folder + map_name_;
  }
}

void MessageProcess::OnLocalization(const LocalizationEstimate& le) {
  if (last_localization_message_timestamp_sec_ == 0.0) {
    last_localization_message_timestamp_sec_ = le.header().timestamp_sec();
  }
  const double time_diff =
      le.header().timestamp_sec() - last_localization_message_timestamp_sec_;
  if (time_diff < 1.0 / FLAGS_planning_loop_rate) {
    // for RL_TEST, E2E_TEST or HYBRID_TEST skip this check so that first
    // frame can proceed
    if (!(planning_config_.learning_mode() == PlanningConfig::RL_TEST ||
          planning_config_.learning_mode() == PlanningConfig::E2E_TEST ||
          planning_config_.learning_mode() == PlanningConfig::HYBRID_TEST)) {
      return;
    }
  }
  if (time_diff >= (1.0 * 2 / FLAGS_planning_loop_rate)) {
    const std::string msg = absl::StrCat(
        "missing localization too long: time_stamp[",
        le.header().timestamp_sec(),  "] time_diff[", time_diff, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }
  last_localization_message_timestamp_sec_ = le.header().timestamp_sec();
  localizations_.push_back(le);

  while (!localizations_.empty()) {
    if (localizations_.back().header().timestamp_sec() -
            localizations_.front().header().timestamp_sec() <=
        FLAGS_trajectory_time_length) {
      break;
    }
    localizations_.pop_front();
  }

  ADEBUG << "OnLocalization: size[" << localizations_.size() << "] time_diff["
         << localizations_.back().header().timestamp_sec() -
                localizations_.front().header().timestamp_sec()
         << "]";

  // generate one frame data
  LearningDataFrame learning_data_frame;
  if (GenerateLearningDataFrame(&learning_data_frame)) {
    // output
    if (FLAGS_planning_offline_learning) {
      // offline
      FeatureOutput::InsertLearningDataFrame(record_file_, learning_data_frame);
    } else {
      // online
      injector_->learning_based_data()->InsertLearningDataFrame(
          learning_data_frame);
    }
  }
}

void MessageProcess::OnPrediction(
    const PredictionObstacles& prediction_obstacles) {
  prediction_obstacles_map_.clear();
  for (int i = 0; i < prediction_obstacles.prediction_obstacle_size(); ++i) {
    const auto& prediction_obstacle =
        prediction_obstacles.prediction_obstacle(i);
    const int obstacle_id = prediction_obstacle.perception_obstacle().id();
    prediction_obstacles_map_[obstacle_id].CopyFrom(prediction_obstacle);
  }

  // erase obstacle history if obstacle not exist in current predictio msg
  /* comment out to relax this check for now
  std::unordered_map<int, std::list<PerceptionObstacleFeature>>::iterator
      it = obstacle_history_map_.begin();
  while (it != obstacle_history_map_.end()) {
    const int obstacle_id = it->first;
    if (prediction_obstacles_map_.count(obstacle_id) == 0) {
      // not exist in current prediction msg
      it = obstacle_history_map_.erase(it);
    } else {
      ++it;
    }
  }
  */

  // debug
  // add to obstacle history
  // for (const auto& m : obstacle_history_map_) {
  //  for (const auto& p :  m.second) {
  //    AERROR << "obstacle_history_map_: " << m.first << "; "
  //           << p.DebugString();
  //  }
  // }

  // add to obstacle history
  for (const auto& m : prediction_obstacles_map_) {
    const auto& perception_obstale = m.second.perception_obstacle();
    PerceptionObstacleFeature obstacle_trajectory_point;
    obstacle_trajectory_point.set_timestamp_sec(perception_obstale.timestamp());
    obstacle_trajectory_point.mutable_position()->CopyFrom(
        perception_obstale.position());
    obstacle_trajectory_point.set_theta(perception_obstale.theta());
    obstacle_trajectory_point.mutable_velocity()->CopyFrom(
        perception_obstale.velocity());
    for (int j = 0; j < perception_obstale.polygon_point_size(); ++j) {
      auto polygon_point = obstacle_trajectory_point.add_polygon_point();
      polygon_point->CopyFrom(perception_obstale.polygon_point(j));
    }
    obstacle_trajectory_point.mutable_acceleration()->CopyFrom(
        perception_obstale.acceleration());

    obstacle_history_map_[m.first].back().timestamp_sec();
    if (obstacle_history_map_[m.first].empty() ||
        obstacle_trajectory_point.timestamp_sec() -
                obstacle_history_map_[m.first].back().timestamp_sec() >
            0) {
      obstacle_history_map_[m.first].push_back(obstacle_trajectory_point);
    } else {
      // abnormal perception data: time_diff <= 0
      const double time_diff =
          obstacle_trajectory_point.timestamp_sec() -
          obstacle_history_map_[m.first].back().timestamp_sec();
      const std::string msg = absl::StrCat(
          "DISCARD: obstacle_id[", m.first, "] last_timestamp_sec[",
          obstacle_history_map_[m.first].back().timestamp_sec(),
          "] timestamp_sec[", obstacle_trajectory_point.timestamp_sec(),
          "] time_diff[",  time_diff,  "]");
      AERROR << msg;
      if (FLAGS_planning_offline_learning) {
        log_file_ << msg << std::endl;
      }
    }
    auto& obstacle_history = obstacle_history_map_[m.first];
    while (!obstacle_history.empty()) {
      const double time_distance = obstacle_history.back().timestamp_sec() -
                                   obstacle_history.front().timestamp_sec();
      if (time_distance < FLAGS_learning_data_obstacle_history_time_sec) {
        break;
      }
      obstacle_history.pop_front();
    }
  }
}

void MessageProcess::OnRoutingResponse(
    const apollo::routing::RoutingResponse& routing_response) {
  ADEBUG << "routing_response received at frame["
         << total_learning_data_frame_num_ << "]";
  routing_response_.CopyFrom(routing_response);
}

void MessageProcess::OnStoryTelling(
    const apollo::storytelling::Stories& stories) {
  // clear area
  if (stories.has_close_to_clear_area()) {
    auto clear_area_tag = planning_tag_.mutable_clear_area();
    clear_area_tag->set_id(stories.close_to_clear_area().id());
    clear_area_tag->set_distance(stories.close_to_clear_area().distance());
  }

  // crosswalk
  if (stories.has_close_to_crosswalk()) {
    auto crosswalk_tag = planning_tag_.mutable_crosswalk();
    crosswalk_tag->set_id(stories.close_to_crosswalk().id());
    crosswalk_tag->set_distance(stories.close_to_crosswalk().distance());
  }

  // pnc_junction
  if (stories.has_close_to_junction() &&
      stories.close_to_junction().type() == CloseToJunction::PNC_JUNCTION) {
    auto pnc_junction_tag = planning_tag_.mutable_pnc_junction();
    pnc_junction_tag->set_id(stories.close_to_junction().id());
    pnc_junction_tag->set_distance(stories.close_to_junction().distance());
  }

  // traffic_light
  if (stories.has_close_to_signal()) {
    auto signal_tag = planning_tag_.mutable_signal();
    signal_tag->set_id(stories.close_to_signal().id());
    signal_tag->set_distance(stories.close_to_signal().distance());
  }

  // stop_sign
  if (stories.has_close_to_stop_sign()) {
    auto stop_sign_tag = planning_tag_.mutable_stop_sign();
    stop_sign_tag->set_id(stories.close_to_stop_sign().id());
    stop_sign_tag->set_distance(stories.close_to_stop_sign().distance());
  }

  // yield_sign
  if (stories.has_close_to_yield_sign()) {
    auto yield_sign_tag = planning_tag_.mutable_yield_sign();
    yield_sign_tag->set_id(stories.close_to_yield_sign().id());
    yield_sign_tag->set_distance(stories.close_to_yield_sign().distance());
  }

  ADEBUG << planning_tag_.DebugString();
}

void MessageProcess::OnTrafficLightDetection(
    const TrafficLightDetection& traffic_light_detection) {
  // AINFO << "traffic_light_detection received at frame["
  //      << total_learning_data_frame_num_ << "]";
  traffic_light_detection_message_timestamp_ =
      traffic_light_detection.header().timestamp_sec();
  traffic_lights_.clear();
  for (int i = 0; i < traffic_light_detection.traffic_light_size(); ++i) {
    TrafficLightFeature traffic_light;
    traffic_light.set_color(traffic_light_detection.traffic_light(i).color());
    traffic_light.set_id(traffic_light_detection.traffic_light(i).id());
    traffic_light.set_confidence(
        traffic_light_detection.traffic_light(i).confidence());
    traffic_light.set_tracking_time(
        traffic_light_detection.traffic_light(i).tracking_time());
    traffic_light.set_remaining_time(
        traffic_light_detection.traffic_light(i).remaining_time());
    traffic_lights_.push_back(traffic_light);
  }
}

void MessageProcess::ProcessOfflineData(const std::string& record_file) {
  log_file_ << "Processing: " << record_file << std::endl;
  record_file_ = record_file;

  RecordReader reader(record_file);
  if (!reader.IsValid()) {
    AERROR << "Fail to open " << record_file;
    return;
  }

  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name ==
        planning_config_.topic_config().chassis_topic()) {
      Chassis chassis;
      if (chassis.ParseFromString(message.content)) {
        OnChassis(chassis);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().localization_topic()) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        OnLocalization(localization);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().hmi_status_topic()) {
      HMIStatus hmi_status;
      if (hmi_status.ParseFromString(message.content)) {
        OnHMIStatus(hmi_status);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().prediction_topic()) {
      PredictionObstacles prediction_obstacles;
      if (prediction_obstacles.ParseFromString(message.content)) {
        OnPrediction(prediction_obstacles);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().routing_response_topic()) {
      RoutingResponse routing_response;
      if (routing_response.ParseFromString(message.content)) {
        OnRoutingResponse(routing_response);
      }
    } else if (message.channel_name ==
               planning_config_.topic_config().story_telling_topic()) {
      Stories stories;
      if (stories.ParseFromString(message.content)) {
        OnStoryTelling(stories);
      }
    } else if (message.channel_name == planning_config_.topic_config()
                                           .traffic_light_detection_topic()) {
      TrafficLightDetection traffic_light_detection;
      if (traffic_light_detection.ParseFromString(message.content)) {
        OnTrafficLightDetection(traffic_light_detection);
      }
    }
  }
}

bool MessageProcess::GetADCCurrentRoutingIndex(
    int* adc_road_index, int* adc_passage_index, double* adc_passage_s) {
  if (localizations_.empty()) return false;

  static constexpr double kRadius = 4.0;
  const auto& pose = localizations_.back().pose();
  std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>> lanes;
  apollo::hdmap::HDMapUtil::BaseMapPtr()->GetLanes(pose.position(), kRadius,
                                                   &lanes);

  for (auto& lane : lanes) {
    for (int i = 0; i < routing_response_.road_size(); ++i) {
      *adc_passage_s = 0;
      for (int j = 0; j < routing_response_.road(i).passage_size(); ++j) {
        double passage_s = 0;
        for (int k = 0; k < routing_response_.road(i).passage(j).segment_size();
             ++k) {
          const auto& segment = routing_response_.road(i).passage(j).segment(k);
          passage_s += (segment.end_s() - segment.start_s());
          if (lane->id().id() == segment.id()) {
            *adc_road_index = i;
            *adc_passage_index = j;
            *adc_passage_s = passage_s;
            return true;
          }
        }
      }
    }
  }
  return false;
}

apollo::hdmap::LaneInfoConstPtr MessageProcess::GetCurrentLane(
    const apollo::common::PointENU& position) {
  constexpr double kRadiusUnit = 0.1;
  std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>> lanes;
  for (int i = 1; i <= 10; ++i) {
    apollo::hdmap::HDMapUtil::BaseMapPtr()->GetLanes(position, i * kRadiusUnit,
                                                     &lanes);
    if (lanes.size() > 0) {
      break;
    }
  }

  for (auto& lane : lanes) {
    for (int i = 0; i < routing_response_.road_size(); ++i) {
      for (int j = 0; j < routing_response_.road(i).passage_size(); ++j) {
        for (int k = 0; k < routing_response_.road(i).passage(j).segment_size();
             ++k) {
          if (lane->id().id() ==
              routing_response_.road(i).passage(j).segment(k).id()) {
            return lane;
          }
        }
      }
    }
  }

  return nullptr;
}

int MessageProcess::GetADCCurrentInfo(ADCCurrentInfo* adc_curr_info) {
  CHECK_NOTNULL(adc_curr_info);
  if (localizations_.empty()) return -1;

  // ADC current position / velocity / acc/ heading
  const auto& adc_cur_pose = localizations_.back().pose();
  adc_curr_info->adc_cur_position_ =
      std::make_pair(adc_cur_pose.position().x(), adc_cur_pose.position().y());
  adc_curr_info->adc_cur_velocity_ = std::make_pair(
      adc_cur_pose.linear_velocity().x(), adc_cur_pose.linear_velocity().y());
  adc_curr_info->adc_cur_acc_ =
      std::make_pair(adc_cur_pose.linear_acceleration().x(),
                     adc_cur_pose.linear_acceleration().y());
  adc_curr_info->adc_cur_heading_ = adc_cur_pose.heading();
  return 1;
}

void MessageProcess::GenerateObstacleTrajectory(
    const int frame_num, const int obstacle_id,
    const ADCCurrentInfo& adc_curr_info, ObstacleFeature* obstacle_feature) {
  auto obstacle_trajectory = obstacle_feature->mutable_obstacle_trajectory();
  const auto& obstacle_history = obstacle_history_map_[obstacle_id];
  for (const auto& obj_traj_point : obstacle_history) {
    auto perception_obstacle_history =
        obstacle_trajectory->add_perception_obstacle_history();
    perception_obstacle_history->set_timestamp_sec(
        obj_traj_point.timestamp_sec());

    // convert position to relative coordinate
    const auto& relative_posistion = util::WorldCoordToObjCoord(
        std::make_pair(obj_traj_point.position().x(),
                       obj_traj_point.position().y()),
        adc_curr_info.adc_cur_position_, adc_curr_info.adc_cur_heading_);
    auto position = perception_obstacle_history->mutable_position();
    position->set_x(relative_posistion.first);
    position->set_y(relative_posistion.second);

    // convert theta to relative coordinate
    const double relative_theta = util::WorldAngleToObjAngle(
        obj_traj_point.theta(), adc_curr_info.adc_cur_heading_);
    perception_obstacle_history->set_theta(relative_theta);

    // convert velocity to relative coordinate
    const auto& relative_velocity = util::WorldCoordToObjCoord(
        std::make_pair(obj_traj_point.velocity().x(),
                       obj_traj_point.velocity().y()),
        adc_curr_info.adc_cur_velocity_, adc_curr_info.adc_cur_heading_);
    auto velocity = perception_obstacle_history->mutable_velocity();
    velocity->set_x(relative_velocity.first);
    velocity->set_y(relative_velocity.second);

    // convert acceleration to relative coordinate
    const auto& relative_acc = util::WorldCoordToObjCoord(
        std::make_pair(obj_traj_point.acceleration().x(),
                       obj_traj_point.acceleration().y()),
        adc_curr_info.adc_cur_acc_, adc_curr_info.adc_cur_heading_);
    auto acceleration = perception_obstacle_history->mutable_acceleration();
    acceleration->set_x(relative_acc.first);
    acceleration->set_y(relative_acc.second);

    for (int i = 0; i < obj_traj_point.polygon_point_size(); ++i) {
      // convert polygon_point(s) to relative coordinate
      const auto& relative_point = util::WorldCoordToObjCoord(
          std::make_pair(obj_traj_point.polygon_point(i).x(),
                         obj_traj_point.polygon_point(i).y()),
          adc_curr_info.adc_cur_position_, adc_curr_info.adc_cur_heading_);
      auto polygon_point = perception_obstacle_history->add_polygon_point();
      polygon_point->set_x(relative_point.first);
      polygon_point->set_y(relative_point.second);
    }
  }
}

void MessageProcess::GenerateObstaclePrediction(
    const int frame_num,
    const PredictionObstacle& prediction_obstacle,
    const ADCCurrentInfo& adc_curr_info, ObstacleFeature* obstacle_feature) {
  const auto obstacle_id = obstacle_feature->id();
  auto obstacle_prediction = obstacle_feature->mutable_obstacle_prediction();
  obstacle_prediction->set_timestamp_sec(prediction_obstacle.timestamp());
  obstacle_prediction->set_predicted_period(
      prediction_obstacle.predicted_period());
  obstacle_prediction->mutable_intent()->CopyFrom(prediction_obstacle.intent());
  obstacle_prediction->mutable_priority()->CopyFrom(
      prediction_obstacle.priority());
  obstacle_prediction->set_is_static(prediction_obstacle.is_static());

  for (int i = 0; i < prediction_obstacle.trajectory_size(); ++i) {
    const auto& obstacle_trajectory = prediction_obstacle.trajectory(i);
    auto trajectory = obstacle_prediction->add_trajectory();
    trajectory->set_probability(obstacle_trajectory.probability());

    // TrajectoryPoint
    for (int j = 0; j < obstacle_trajectory.trajectory_point_size(); ++j) {
      const auto& obstacle_trajectory_point =
          obstacle_trajectory.trajectory_point(j);

      if (trajectory->trajectory_point_size() >0) {
        const auto last_relative_time =
            trajectory->trajectory_point(trajectory->trajectory_point_size()-1)
                .trajectory_point().relative_time();
        if (obstacle_trajectory_point.relative_time() < last_relative_time) {
          const std::string msg = absl::StrCat(
              "DISCARD prediction trajectory point: frame_num[", frame_num,
              "] obstacle_id[", obstacle_id, "] last_relative_time[",
              last_relative_time, "] relative_time[",
              obstacle_trajectory_point.relative_time(), "]");
          AERROR << msg;
          if (FLAGS_planning_offline_learning) {
            log_file_ << msg << std::endl;
          }
          continue;
        }
      }

      auto trajectory_point = trajectory->add_trajectory_point();

      auto path_point =
          trajectory_point->mutable_trajectory_point()->mutable_path_point();

      // convert path_point position to relative coordinate
      const auto& relative_path_point = util::WorldCoordToObjCoord(
          std::make_pair(obstacle_trajectory_point.path_point().x(),
                         obstacle_trajectory_point.path_point().y()),
          adc_curr_info.adc_cur_position_, adc_curr_info.adc_cur_heading_);
      path_point->set_x(relative_path_point.first);
      path_point->set_y(relative_path_point.second);

      // convert path_point theta to relative coordinate
      const double relative_theta = util::WorldAngleToObjAngle(
          obstacle_trajectory_point.path_point().theta(),
          adc_curr_info.adc_cur_heading_);
      path_point->set_theta(relative_theta);

      path_point->set_s(obstacle_trajectory_point.path_point().s());
      path_point->set_lane_id(obstacle_trajectory_point.path_point().lane_id());

      const double timestamp_sec = prediction_obstacle.timestamp() +
                                   obstacle_trajectory_point.relative_time();
      trajectory_point->set_timestamp_sec(timestamp_sec);
      auto tp = trajectory_point->mutable_trajectory_point();
      tp->set_v(obstacle_trajectory_point.v());
      tp->set_a(obstacle_trajectory_point.a());
      tp->set_relative_time(obstacle_trajectory_point.relative_time());
      tp->mutable_gaussian_info()->CopyFrom(
          obstacle_trajectory_point.gaussian_info());
    }
  }
}

void MessageProcess::GenerateObstacleFeature(
    LearningDataFrame* learning_data_frame) {
  ADCCurrentInfo adc_curr_info;
  if (GetADCCurrentInfo(&adc_curr_info) == -1) {
    const std::string msg = absl::StrCat(
        "fail to get ADC current info: frame_num[",
        learning_data_frame->frame_num(), "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
    return;
  }

  const int frame_num = learning_data_frame->frame_num();
  for (const auto& m : prediction_obstacles_map_) {
    auto obstacle_feature = learning_data_frame->add_obstacle();

    const auto& perception_obstale = m.second.perception_obstacle();
    obstacle_feature->set_id(m.first);
    obstacle_feature->set_length(perception_obstale.length());
    obstacle_feature->set_width(perception_obstale.width());
    obstacle_feature->set_height(perception_obstale.height());
    obstacle_feature->set_type(perception_obstale.type());

    // obstacle history trajectory points
    GenerateObstacleTrajectory(frame_num, m.first, adc_curr_info,
                               obstacle_feature);

    // obstacle prediction
    GenerateObstaclePrediction(frame_num, m.second, adc_curr_info,
                               obstacle_feature);
  }
}

bool MessageProcess::GenerateLocalRouting(
    const int frame_num,
    RoutingResponseFeature* local_routing,
    std::vector<std::string>* local_routing_lane_ids) {
  local_routing->Clear();
  local_routing_lane_ids->clear();

  if (routing_response_.road_size() == 0 ||
      routing_response_.road(0).passage_size() == 0 ||
      routing_response_.road(0).passage(0).segment_size() == 0) {
    const std::string msg = absl::StrCat(
        "DISCARD: invalid routing_response. frame_num[", frame_num, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
    return false;
  }

  // calculate road_length
  std::vector<std::pair<std::string, double>> road_lengths;
  for (int i = 0; i < routing_response_.road_size(); ++i) {
    ADEBUG << "road_id[" << routing_response_.road(i).id() << "] passage_size["
           << routing_response_.road(i).passage_size() << "]";
    double road_length = 0.0;
    for (int j = 0; j < routing_response_.road(i).passage_size(); ++j) {
      ADEBUG << "   passage: segment_size["
             << routing_response_.road(i).passage(j).segment_size() << "]";
      double passage_length = 0;
      for (int k = 0; k < routing_response_.road(i).passage(j).segment_size();
           ++k) {
        const auto& segment = routing_response_.road(i).passage(j).segment(k);
        passage_length += (segment.end_s() - segment.start_s());
      }
      ADEBUG << "      passage_length[" << passage_length << "]";
      road_length = std::max(road_length, passage_length);
    }

    road_lengths.push_back(
        std::make_pair(routing_response_.road(i).id(), road_length));
    ADEBUG << "   road_length[" << road_length << "]";
  }

  /* debug
  for (size_t i = 0; i < road_lengths.size(); ++i) {
    AERROR << i << ": " << road_lengths[i].first << "; "
           << road_lengths[i].second;
  }
  */

  int adc_road_index;
  int adc_passage_index;
  double adc_passage_s;
  if (!GetADCCurrentRoutingIndex(&adc_road_index, &adc_passage_index,
                                 &adc_passage_s) ||
      adc_road_index < 0 || adc_passage_index < 0 || adc_passage_s < 0) {
    // reset localization history
    localizations_.clear();

    const std::string msg = absl::StrCat(
        "DISCARD: fail to locate ADC on routing. frame_num[",
        frame_num, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
    return false;
  }
  ADEBUG << "adc_road_index[" << adc_road_index << "] adc_passage_index["
         << adc_passage_index << "] adc_passage_s[" << adc_passage_s << "]";

  constexpr double kLocalRoutingForwardLength = 200.0;
  constexpr double kLocalRoutingBackwardLength = 100.0;

  // local_routing start point
  int local_routing_start_road_index = 0;
  double local_routing_start_road_s = 0;
  double backward_length = kLocalRoutingBackwardLength;
  for (int i = adc_road_index; i >= 0; --i) {
    const double road_length =
        (i == adc_road_index ? adc_passage_s : road_lengths[i].second);
    if (backward_length > road_length) {
      backward_length -= road_length;
    } else {
      local_routing_start_road_index = i;
      local_routing_start_road_s = road_length - backward_length;
      ADEBUG << "local_routing_start_road_index["
             << local_routing_start_road_index
             << "] local_routing_start_road_s["
             << local_routing_start_road_s << "]";
      break;
    }
  }

  // local routing end point
  int local_routing_end_road_index = routing_response_.road_size() - 1;
  double local_routing_end_road_s =
      road_lengths[local_routing_end_road_index].second;
  double forwardward_length = kLocalRoutingForwardLength;
  for (int i = adc_road_index; i < routing_response_.road_size(); ++i) {
    const double road_length =
        (i == adc_road_index ? road_lengths[i].second - adc_passage_s
                             : road_lengths[i].second);
    if (forwardward_length > road_length) {
      forwardward_length -= road_length;
    } else {
      local_routing_end_road_index = i;
      local_routing_end_road_s =
          (i == adc_road_index ? adc_passage_s + forwardward_length
                               : forwardward_length);
      ADEBUG << "local_routing_end_road_index[" << local_routing_end_road_index
             << "] local_routing_end_road_s["
             << local_routing_end_road_s << "]";
      break;
    }
  }

  ADEBUG << "local_routing: start_road_index[" << local_routing_start_road_index
         << "] start_road_s[" << local_routing_start_road_s
         << "] end_road_index[" << local_routing_end_road_index
         << "] end_road_s[" << local_routing_end_road_s << "]";

  bool local_routing_end = false;
  int last_passage_index = adc_passage_index;
  for (int i = local_routing_start_road_index;
       i <= local_routing_end_road_index; ++i) {
    if (local_routing_end) break;

    const auto& road = routing_response_.road(i);
    auto local_routing_road = local_routing->add_road();
    local_routing_road->set_id(road.id());

    for (int j = 0; j < road.passage_size(); ++j) {
      const auto& passage = road.passage(j);
      auto local_routing_passage = local_routing_road->add_passage();
      local_routing_passage->set_can_exit(passage.can_exit());
      local_routing_passage->set_change_lane_type(passage.change_lane_type());

      double road_s = 0;
      for (int k = 0; k < passage.segment_size(); ++k) {
        const auto& lane_segment = passage.segment(k);
        road_s += (lane_segment.end_s() - lane_segment.start_s());

        // first road
        if (i == local_routing_start_road_index &&
            road_s < local_routing_start_road_s) {
          continue;
        }

        local_routing_passage->add_segment()->CopyFrom(lane_segment);
        ADEBUG << "ADD road[" << i << "] id[" << road.id() << "] passage[" << j
               << "] id[" << lane_segment.id() << "] length["
               << lane_segment.end_s() - lane_segment.start_s() << "]";

        // set local_routing_lane_ids
        if (i == adc_road_index) {
          // adc_road_index, pick the passage where ADC is
          if (j == adc_passage_index) {
            local_routing_lane_ids->push_back(lane_segment.id());
            ADEBUG << "ADD local_routing_lane_ids: road[" << i
                   << "] passage[" << j << "]: " << lane_segment.id();
            last_passage_index = j;
          }
        } else {
          if (road.passage_size() == 1) {
            // single passage
            ADEBUG << "ADD local_routing_lane_ids: road[" << i
                   << "] passage[" << j << "]: " << lane_segment.id();
            local_routing_lane_ids->push_back(lane_segment.id());
            last_passage_index = j;
          } else {
            // multi passages
            if (i < adc_road_index) {
              // road behind ADC position
              if (j == adc_passage_index ||
                  (j == road.passage_size() - 1 &&
                  j < adc_passage_index)) {
                ADEBUG << "ADD local_routing_lane_ids: road[" << i
                       << "] passage[" << j << "] adc_passage_index["
                       << adc_passage_index << "] passage_size["
                       << road.passage_size() << "]: " << lane_segment.id();
                local_routing_lane_ids->push_back(lane_segment.id());
              }
            } else {
              // road in front of ADC position:
              // pick the passage towards change-left
              if (j == last_passage_index + 1 ||
                  (j == road.passage_size() - 1 &&
                  j < last_passage_index + 1)) {
                ADEBUG << "ADD local_routing_lane_ids: road[" << i
                       << "] passage[" << j << "] last_passage_index["
                       << last_passage_index << "] passage_size["
                       << road.passage_size() << "]: " << lane_segment.id();
                local_routing_lane_ids->push_back(lane_segment.id());
                last_passage_index = j;
              }
            }
          }
        }

        // last road
        if (i == local_routing_end_road_index &&
            road_s >= local_routing_end_road_s) {
          local_routing_end = true;
          break;
        }
      }
    }
  }

  // check local_routing: to filter out map mismatching frames
  if (FLAGS_planning_offline_learning) {
    for (size_t i = 0; i < local_routing_lane_ids->size(); ++i)  {
      const std::string lane_id = local_routing_lane_ids->at(i);
      const auto& lane =
          hdmap::HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(lane_id));
      if (lane == nullptr) {
        const std::string msg = absl::StrCat(
            "DISCARD: fail to find local_routing_lane on map. frame_num[",
            frame_num, "] lane[", lane_id, "]");
        AERROR << msg;
        if (FLAGS_planning_offline_learning) {
          log_file_ << msg << std::endl;
        }
        return false;
      }
    }
  }

  return true;
}

void MessageProcess::GenerateRoutingFeature(
    const RoutingResponseFeature& local_routing,
    const std::vector<std::string>& local_routing_lane_ids,
    LearningDataFrame* learning_data_frame) {
  auto routing = learning_data_frame->mutable_routing();
  routing->Clear();

  routing->mutable_routing_response()->mutable_measurement()->set_distance(
      routing_response_.measurement().distance());
  for (int i = 0; i < routing_response_.road_size(); ++i) {
    routing->mutable_routing_response()->add_road()->CopyFrom(
        routing_response_.road(i));
  }

  for (const auto& lane_id : local_routing_lane_ids) {
    routing->add_local_routing_lane_id(lane_id);
  }
  routing->mutable_local_routing()->CopyFrom(local_routing);

  const int frame_num = learning_data_frame->frame_num();
  const int local_routing_lane_id_size = routing->local_routing_lane_id_size();
  if (local_routing_lane_id_size == 0) {
    const std::string msg = absl::StrCat(
        "empty local_routing. frame_num[", frame_num, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }
  if (local_routing_lane_id_size > 100) {
    const std::string msg = absl::StrCat(
        "LARGE local_routing. frame_num[", frame_num,
        "] local_routing_lane_id_size[", local_routing_lane_id_size, "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }
  ADEBUG << "local_routing: frame_num[" << frame_num
         << "] size[" << routing->local_routing_lane_id_size() << "]";
}

void MessageProcess::GenerateTrafficLightDetectionFeature(
    LearningDataFrame* learning_data_frame) {
  auto traffic_light_detection =
      learning_data_frame->mutable_traffic_light_detection();
  traffic_light_detection->set_message_timestamp_sec(
      traffic_light_detection_message_timestamp_);
  traffic_light_detection->clear_traffic_light();
  for (const auto& tl : traffic_lights_) {
    auto traffic_light = traffic_light_detection->add_traffic_light();
    traffic_light->CopyFrom(tl);
  }
}

void MessageProcess::GenerateADCTrajectoryPoints(
    const std::list<LocalizationEstimate>& localizations,
    LearningDataFrame* learning_data_frame) {
  std::vector<LocalizationEstimate> localization_samples;
  for (const auto& le : localizations) {
    localization_samples.insert(localization_samples.begin(), le);
  }

  constexpr double kSearchRadius = 1.0;

  std::string clear_area_id;
  double clear_area_distance = 0.0;
  std::string crosswalk_id;
  double crosswalk_distance = 0.0;
  std::string pnc_junction_id;
  double pnc_junction_distance = 0.0;
  std::string signal_id;
  double signal_distance = 0.0;
  std::string stop_sign_id;
  double stop_sign_distance = 0.0;
  std::string yield_sign_id;
  double yield_sign_distance = 0.0;

  int trajectory_point_index = 0;
  std::vector<ADCTrajectoryPoint> adc_trajectory_points;
  for (const auto& localization_sample : localization_samples) {
    ADCTrajectoryPoint adc_trajectory_point;
    adc_trajectory_point.set_timestamp_sec(
        localization_sample.measurement_time());

    auto trajectory_point = adc_trajectory_point.mutable_trajectory_point();
    auto& pose = localization_sample.pose();
    trajectory_point->mutable_path_point()->set_x(pose.position().x());
    trajectory_point->mutable_path_point()->set_y(pose.position().y());
    trajectory_point->mutable_path_point()->set_z(pose.position().z());
    trajectory_point->mutable_path_point()->set_theta(pose.heading());

    const double v =
        std::sqrt(pose.linear_velocity().x() * pose.linear_velocity().x() +
                  pose.linear_velocity().y() * pose.linear_velocity().y());
    trajectory_point->set_v(v);

    const double a = std::sqrt(
        pose.linear_acceleration().x() * pose.linear_acceleration().x() +
        pose.linear_acceleration().y() * pose.linear_acceleration().y());
    trajectory_point->set_a(a);

    auto planning_tag = adc_trajectory_point.mutable_planning_tag();

    // planning_tag: lane_turn
    const auto& cur_point = common::util::PointFactory::ToPointENU(
        pose.position().x(), pose.position().y(), pose.position().z());
    LaneInfoConstPtr lane = GetCurrentLane(cur_point);

    // lane_turn
    apollo::hdmap::Lane::LaneTurn lane_turn = apollo::hdmap::Lane::NO_TURN;
    if (lane != nullptr) {
      lane_turn = lane->lane().turn();
    }
    planning_tag->set_lane_turn(lane_turn);
    planning_tag_.set_lane_turn(lane_turn);

    if (FLAGS_planning_offline_learning) {
      // planning_tag: overlap tags
      double point_distance = 0.0;
      if (trajectory_point_index > 0) {
        auto& next_point = adc_trajectory_points[trajectory_point_index - 1]
                               .trajectory_point()
                               .path_point();
        point_distance = common::util::DistanceXY(next_point, cur_point);
      }

      common::PointENU hdmap_point;
      hdmap_point.set_x(cur_point.x());
      hdmap_point.set_y(cur_point.y());

      // clear area
      planning_tag->clear_clear_area();
      std::vector<ClearAreaInfoConstPtr> clear_areas;
      if (HDMapUtil::BaseMap().GetClearAreas(hdmap_point, kSearchRadius,
                                             &clear_areas) == 0 &&
          clear_areas.size() > 0) {
        clear_area_id = clear_areas.front()->id().id();
        clear_area_distance = 0.0;
      } else {
        if (!clear_area_id.empty()) {
          clear_area_distance += point_distance;
        }
      }
      if (!clear_area_id.empty()) {
        planning_tag->mutable_clear_area()->set_id(clear_area_id);
        planning_tag->mutable_clear_area()->set_distance(clear_area_distance);
      }

      // crosswalk
      planning_tag->clear_crosswalk();
      std::vector<CrosswalkInfoConstPtr> crosswalks;
      if (HDMapUtil::BaseMap().GetCrosswalks(hdmap_point, kSearchRadius,
                                             &crosswalks) == 0 &&
          crosswalks.size() > 0) {
        crosswalk_id = crosswalks.front()->id().id();
        crosswalk_distance = 0.0;
      } else {
        if (!crosswalk_id.empty()) {
          crosswalk_distance += point_distance;
        }
      }
      if (!crosswalk_id.empty()) {
        planning_tag->mutable_crosswalk()->set_id(crosswalk_id);
        planning_tag->mutable_crosswalk()->set_distance(crosswalk_distance);
      }

      // pnc_junction
      std::vector<PNCJunctionInfoConstPtr> pnc_junctions;
      if (HDMapUtil::BaseMap().GetPNCJunctions(hdmap_point, kSearchRadius,
                                               &pnc_junctions) == 0 &&
          pnc_junctions.size() > 0) {
        pnc_junction_id = pnc_junctions.front()->id().id();
        pnc_junction_distance = 0.0;
      } else {
        if (!pnc_junction_id.empty()) {
          pnc_junction_distance += point_distance;
        }
      }
      if (!pnc_junction_id.empty()) {
        planning_tag->mutable_pnc_junction()->set_id(pnc_junction_id);
        planning_tag->mutable_pnc_junction()->set_distance(
            pnc_junction_distance);
      }

      // signal
      std::vector<SignalInfoConstPtr> signals;
      if (HDMapUtil::BaseMap().GetSignals(hdmap_point, kSearchRadius,
                                          &signals) == 0 &&
          signals.size() > 0) {
        signal_id = signals.front()->id().id();
        signal_distance = 0.0;
      } else {
        if (!signal_id.empty()) {
          signal_distance += point_distance;
        }
      }
      if (!signal_id.empty()) {
        planning_tag->mutable_signal()->set_id(signal_id);
        planning_tag->mutable_signal()->set_distance(signal_distance);
      }

      // stop sign
      std::vector<StopSignInfoConstPtr> stop_signs;
      if (HDMapUtil::BaseMap().GetStopSigns(hdmap_point, kSearchRadius,
                                            &stop_signs) == 0 &&
          stop_signs.size() > 0) {
        stop_sign_id = stop_signs.front()->id().id();
        stop_sign_distance = 0.0;
      } else {
        if (!stop_sign_id.empty()) {
          stop_sign_distance += point_distance;
        }
      }
      if (!stop_sign_id.empty()) {
        planning_tag->mutable_stop_sign()->set_id(stop_sign_id);
        planning_tag->mutable_stop_sign()->set_distance(stop_sign_distance);
      }

      // yield sign
      std::vector<YieldSignInfoConstPtr> yield_signs;
      if (HDMapUtil::BaseMap().GetYieldSigns(hdmap_point, kSearchRadius,
                                             &yield_signs) == 0 &&
          yield_signs.size() > 0) {
        yield_sign_id = yield_signs.front()->id().id();
        yield_sign_distance = 0.0;
      } else {
        if (!yield_sign_id.empty()) {
          yield_sign_distance += point_distance;
        }
      }
      if (!yield_sign_id.empty()) {
        planning_tag->mutable_yield_sign()->set_id(yield_sign_id);
        planning_tag->mutable_yield_sign()->set_distance(yield_sign_distance);
      }
    }

    adc_trajectory_points.push_back(adc_trajectory_point);
    ++trajectory_point_index;
  }

  // update learning data
  std::reverse(adc_trajectory_points.begin(), adc_trajectory_points.end());
  for (const auto& trajectory_point : adc_trajectory_points) {
    auto adc_trajectory_point = learning_data_frame->add_adc_trajectory_point();
    adc_trajectory_point->CopyFrom(trajectory_point);
  }
  if (adc_trajectory_points.size() <= 5) {
    const std::string msg = absl::StrCat(
        "too few adc_trajectory_points: frame_num[",
        learning_data_frame->frame_num(), "] size[",
        adc_trajectory_points.size(), "]");
    AERROR << msg;
    if (FLAGS_planning_offline_learning) {
      log_file_ << msg << std::endl;
    }
  }
  // AINFO << "number of ADC trajectory points in one frame: "
  //      << trajectory_point_index;
}

void MessageProcess::GeneratePlanningTag(
    LearningDataFrame* learning_data_frame) {
  auto planning_tag = learning_data_frame->mutable_planning_tag();
  if (FLAGS_planning_offline_learning) {
    planning_tag->set_lane_turn(planning_tag_.lane_turn());
  } else {
    if (planning_config_.learning_mode() != PlanningConfig::NO_LEARNING) {
      // online learning
      planning_tag->CopyFrom(planning_tag_);
    }
  }
}

bool MessageProcess::GenerateLearningDataFrame(
    LearningDataFrame* learning_data_frame) {
  const double start_timestamp = Clock::NowInSeconds();

  RoutingResponseFeature local_routing;
  std::vector<std::string> local_routing_lane_ids;
  if (!GenerateLocalRouting(total_learning_data_frame_num_,
                            &local_routing, &local_routing_lane_ids)) {
    return false;
  }

  // add timestamp_sec & frame_num
  learning_data_frame->set_message_timestamp_sec(
      localizations_.back().header().timestamp_sec());
  learning_data_frame->set_frame_num(total_learning_data_frame_num_++);

  // map_name
  learning_data_frame->set_map_name(map_name_);

  // planning_tag
  GeneratePlanningTag(learning_data_frame);

  // add chassis
  auto chassis = learning_data_frame->mutable_chassis();
  chassis->CopyFrom(chassis_feature_);

  // add localization
  auto localization = learning_data_frame->mutable_localization();
  localization->set_message_timestamp_sec(
      localizations_.back().header().timestamp_sec());
  const auto& pose = localizations_.back().pose();
  localization->mutable_position()->CopyFrom(pose.position());
  localization->set_heading(pose.heading());
  localization->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
  localization->mutable_linear_acceleration()->CopyFrom(
      pose.linear_acceleration());
  localization->mutable_angular_velocity()->CopyFrom(pose.angular_velocity());

  // add traffic_light
  GenerateTrafficLightDetectionFeature(learning_data_frame);

  // add routing
  GenerateRoutingFeature(local_routing, local_routing_lane_ids,
                         learning_data_frame);

  // add obstacle
  GenerateObstacleFeature(learning_data_frame);

  // add trajectory_points
  GenerateADCTrajectoryPoints(localizations_, learning_data_frame);

  const double end_timestamp = Clock::NowInSeconds();
  const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  ADEBUG << "MessageProcess: start_timestamp[" << start_timestamp
         << "] end_timestamp[" << end_timestamp << "] time_diff_ms["
         << time_diff_ms << "]";
  return true;
}

}  // namespace planning
}  // namespace apollo
