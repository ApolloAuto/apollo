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

#include "modules/planning/pipeline/feature_generator.h"

#include <cmath>
#include <memory>
#include <string>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/math_util.h"

DEFINE_string(planning_data_dir, "/apollo/modules/planning/data/",
              "Prefix of files to store learning_data_frame data");
DEFINE_int32(localization_freq, 100, "frequence of localization message");
DEFINE_int32(planning_freq, 10, "frequence of planning message");
DEFINE_int32(learning_data_frame_num_per_file, 100,
             "number of learning_data_frame to write out in one data file.");
DEFINE_int32(learning_data_obstacle_history_point_cnt, 20,
             "number of history trajectory points for a obstacle");
DEFINE_bool(enable_binary_learning_data, true,
            "True to generate protobuf binary data file.");
DEFINE_bool(enable_overlap_tag, true,
            "True to add overlap tag to planning_tag");

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
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
using apollo::prediction::PredictionObstacle;
using apollo::prediction::PredictionObstacles;
using apollo::perception::TrafficLightDetection;
using apollo::routing::RoutingResponse;

void FeatureGenerator::Init() {
  map_name_ = "sunnyvale_with_two_offices";
  map_m_["Sunnyvale"] = "sunnyvale";
  map_m_["Sunnyvale Big Loop"] = "sunnyvale_big_loop";
  map_m_["Sunnyvale With Two Offices"] = "sunnyvale_with_two_offices";
  map_m_["Gomentum"] = "gomentum";
  map_m_["Sunnyvale Loop"] = "sunnyvale_loop";
  map_m_["San Mateo"] = "san_mateo";
}

void FeatureGenerator::WriteOutLearningData(
    const LearningData& learning_data,
    const int learning_data_file_index) {
  if (record_file_name_.empty()) {
    record_file_name_ = "00000";
  }
  const std::string file_name = absl::StrCat(
      FLAGS_planning_data_dir, record_file_name_, ".",
      learning_data_file_index, ".bin");
  if (FLAGS_enable_binary_learning_data) {
    cyber::common::SetProtoToBinaryFile(learning_data, file_name);
    cyber::common::SetProtoToASCIIFile(learning_data, file_name + ".txt");
  } else {
    cyber::common::SetProtoToASCIIFile(learning_data, file_name);
  }
  learning_data_.Clear();
  ++learning_data_file_index_;
}

void FeatureGenerator::Close() {
  WriteOutLearningData(learning_data_, learning_data_file_index_);
  AINFO << "Total learning_data_frame number:"
        << total_learning_data_frame_num_;
}

void FeatureGenerator::OnLocalization(const LocalizationEstimate& le) {
  localization_for_label_.push_back(le);

  const int localization_msg_start_cnt =
      FLAGS_localization_freq * FLAGS_trajectory_time_length;
  if (static_cast<int>(localization_for_label_.size()) <
      localization_msg_start_cnt) {
    return;
  }

  // generate one frame data
  GenerateLearningDataFrame();

  const int localization_move_window_step =
      FLAGS_localization_freq / FLAGS_planning_freq;
  for (int i = 0; i < localization_move_window_step; ++i) {
    localization_for_label_.pop_front();
  }

  // write frames into a file
  if (learning_data_.learning_data_size() >=
      FLAGS_learning_data_frame_num_per_file) {
    WriteOutLearningData(learning_data_, learning_data_file_index_);
  }
}

void FeatureGenerator::OnHMIStatus(apollo::dreamview::HMIStatus hmi_status) {
  const std::string& current_map = hmi_status.current_map();
  if (map_m_.count(current_map) > 0) {
    map_name_ = map_m_[current_map];
    const std::string& map_base_folder = "/apollo/modules/map/data/";
    FLAGS_map_dir = map_base_folder + map_name_;
  }
}

void FeatureGenerator::OnChassis(const apollo::canbus::Chassis& chassis) {
  chassis_feature_.set_speed_mps(chassis.speed_mps());
  chassis_feature_.set_throttle_percentage(chassis.throttle_percentage());
  chassis_feature_.set_brake_percentage(chassis.brake_percentage());
  chassis_feature_.set_steering_percentage(chassis.steering_percentage());
  chassis_feature_.set_gear_location(chassis.gear_location());
}

void FeatureGenerator::OnPrediction(
    const PredictionObstacles& prediction_obstacles) {
  prediction_obstacles_map_.clear();
  for (int i = 0; i < prediction_obstacles.prediction_obstacle_size(); ++i) {
    const auto& prediction_obstacle =
        prediction_obstacles.prediction_obstacle(i);
    const int obstacle_id = prediction_obstacle.perception_obstacle().id();
    prediction_obstacles_map_[obstacle_id].CopyFrom(prediction_obstacle);
  }

  // erase obstacle history if obstacle not exist in current predictio msg
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

    obstacle_history_map_[m.first].push_back(obstacle_trajectory_point);

    auto& obstacle_history = obstacle_history_map_[m.first];
    if (static_cast<int>(obstacle_history.size()) >
        FLAGS_learning_data_obstacle_history_point_cnt) {
      obstacle_history.pop_front();
    }
  }
}

void FeatureGenerator::OnTafficLightDetection(
    const TrafficLightDetection& traffic_light_detection) {
  // AINFO << "traffic_light_detection received at frame["
  //      << total_learning_data_frame_num_ << "]";

  traffic_lights_.clear();
  for (int i = 0; i < traffic_light_detection.traffic_light_size(); ++i) {
    const auto& traffic_light_id =
        traffic_light_detection.traffic_light(i).id();
    if (traffic_light_id.empty()) continue;

    // AINFO << "  traffic_light_id[" << traffic_light_id << "] color["
    //       << traffic_light_detection.traffic_light(i).color() << "]";
    traffic_lights_[traffic_light_id] =
        traffic_light_detection.traffic_light(i).color();
  }
}


void FeatureGenerator::OnRoutingResponse(
  const apollo::routing::RoutingResponse& routing_response) {
  AINFO << "routing_response received at frame["
        << total_learning_data_frame_num_ << "]";
  routing_lane_segment_.clear();
  for (int i = 0; i < routing_response.road_size(); ++i) {
    for (int j = 0; j < routing_response.road(i).passage_size(); ++j) {
      for (int k = 0; k < routing_response.road(i).passage(j).segment_size();
          ++k) {
        const auto& lane_segment =
            routing_response.road(i).passage(j).segment(k);
        routing_lane_segment_.push_back(
            std::make_pair(lane_segment.id(),
                lane_segment.end_s() - lane_segment.start_s()));
      }
    }
  }
}

apollo::hdmap::LaneInfoConstPtr FeatureGenerator::GetLane(
    const apollo::common::PointENU& position,
    int* routing_index) {
  constexpr double kRadius = 0.1;
  std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>> lanes;
  for (int i = 0; i < 10; ++i) {
    apollo::hdmap::HDMapUtil::BaseMapPtr()->GetLanes(
        position, kRadius + i * kRadius, &lanes);
    if (lanes.size() > 0) {
      break;
    }
  }

  *routing_index = -1;
  if (lanes.size() >= 0) {
    for (auto& lane : lanes) {
      const auto lane_id = lane->id().id();
      for (size_t i = 0; i < routing_lane_segment_.size(); ++i) {
        if (routing_lane_segment_[i].first == lane_id) {
          *routing_index = i;
          return lane;
        }
      }
    }
  }
  return nullptr;
}

apollo::hdmap::LaneInfoConstPtr FeatureGenerator::GetADCCurrentLane(
    int* routing_index) {
  const auto& pose = localization_for_label_.back().pose();
  return GetLane(pose.position(), routing_index);
}

void FeatureGenerator::GetADCCurrentInfo(ADCCurrentInfo* adc_curr_info) {
  CHECK_NOTNULL(adc_curr_info);
  // ADC current position / velocity / acc/ heading
  const auto& adc_cur_pose = localization_for_label_.back().pose();
  adc_curr_info->adc_cur_position_ =
      std::make_pair(adc_cur_pose.position().x(),
                     adc_cur_pose.position().y());
  adc_curr_info->adc_cur_velocity_ =
      std::make_pair(adc_cur_pose.linear_velocity().x(),
                     adc_cur_pose.linear_velocity().y());
  adc_curr_info->adc_cur_acc_ =
      std::make_pair(adc_cur_pose.linear_acceleration().x(),
                     adc_cur_pose.linear_acceleration().y());
  adc_curr_info->adc_cur_heading_ = adc_cur_pose.heading();
}

void FeatureGenerator::GenerateObstacleTrajectoryPoint(
    const int obstacle_id,
    const ADCCurrentInfo& adc_curr_info,
    ObstacleFeature* obstacle_feature) {
  const auto& obstacle_history = obstacle_history_map_[obstacle_id];
  for (const auto& obj_traj_point : obstacle_history) {
    auto obstacle_trajectory_point =
        obstacle_feature->add_obstacle_trajectory_point();
    obstacle_trajectory_point->set_timestamp_sec(
        obj_traj_point.timestamp_sec());

    // convert position to relative coordinate
    const auto& relative_posistion =
        util::WorldCoordToObjCoord(
            std::make_pair(obj_traj_point.position().x(),
                           obj_traj_point.position().y()),
            adc_curr_info.adc_cur_position_,
            adc_curr_info.adc_cur_heading_);
    auto position = obstacle_trajectory_point->mutable_position();
    position->set_x(relative_posistion.first);
    position->set_y(relative_posistion.second);

    // convert theta to relative coordinate
    const double relative_theta =
        util::WorldAngleToObjAngle(obj_traj_point.theta(),
                                     adc_curr_info.adc_cur_heading_);
    obstacle_trajectory_point->set_theta(relative_theta);

    // convert velocity to relative coordinate
    const auto& relative_velocity =
        util::WorldCoordToObjCoord(
            std::make_pair(obj_traj_point.velocity().x(),
                           obj_traj_point.velocity().y()),
            adc_curr_info.adc_cur_velocity_,
            adc_curr_info.adc_cur_heading_);
    auto velocity = obstacle_trajectory_point->mutable_velocity();
    velocity->set_x(relative_velocity.first);
    velocity->set_y(relative_velocity.second);

    // convert acceleration to relative coordinate
    const auto& relative_acc =
        util::WorldCoordToObjCoord(
            std::make_pair(obj_traj_point.acceleration().x(),
                           obj_traj_point.acceleration().y()),
            adc_curr_info.adc_cur_acc_,
            adc_curr_info.adc_cur_heading_);
    auto acceleration = obstacle_trajectory_point->mutable_acceleration();
    acceleration->set_x(relative_acc.first);
    acceleration->set_y(relative_acc.second);

    for (int i = 0; i < obj_traj_point.polygon_point_size();
        ++i) {
      // convert polygon_point(s) to relative coordinate
      const auto& relative_point =
          util::WorldCoordToObjCoord(
              std::make_pair(obj_traj_point.polygon_point(i).x(),
                             obj_traj_point.polygon_point(i).y()),
              adc_curr_info.adc_cur_position_,
              adc_curr_info.adc_cur_heading_);
      auto polygon_point = obstacle_trajectory_point->add_polygon_point();
      polygon_point->set_x(relative_point.first);
      polygon_point->set_y(relative_point.second);
    }
  }
}

void FeatureGenerator::GenerateObstaclePrediction(
    const PredictionObstacle& prediction_obstacle,
    const ADCCurrentInfo& adc_curr_info,
    ObstacleFeature* obstacle_feature) {
  auto obstacle_prediction = obstacle_feature->mutable_obstacle_prediction();
  obstacle_prediction->set_timestamp_sec(prediction_obstacle.timestamp());
  obstacle_prediction->set_predicted_period(
      prediction_obstacle.predicted_period());
  obstacle_prediction->mutable_intent()->CopyFrom(
      prediction_obstacle.intent());
  obstacle_prediction->mutable_priority()->CopyFrom(
      prediction_obstacle.priority());
  obstacle_prediction->set_is_static(prediction_obstacle.is_static());

  for (int i = 0; i < prediction_obstacle.trajectory_size(); ++i) {
    const auto& obstacle_trajectory = prediction_obstacle.trajectory(i);
    auto trajectory = obstacle_prediction->add_trajectory();
    trajectory->set_probability(obstacle_trajectory.probability());

    for (int j = 0; j < obstacle_trajectory.trajectory_point_size(); ++j) {
      const auto& obstacle_trajectory_point =
          obstacle_trajectory.trajectory_point(j);
      auto trajectory_point = trajectory->add_trajectory_point();

      auto path_point = trajectory_point->mutable_path_point();

      // convert path_point position to relative coordinate
      const auto& relative_path_point =
        util::WorldCoordToObjCoord(
            std::make_pair(obstacle_trajectory_point.path_point().x(),
                           obstacle_trajectory_point.path_point().y()),
            adc_curr_info.adc_cur_position_,
            adc_curr_info.adc_cur_heading_);
      path_point->set_x(relative_path_point.first);
      path_point->set_y(relative_path_point.second);

      // convert path_point theta to relative coordinate
      const double relative_theta =
          util::WorldAngleToObjAngle(
              obstacle_trajectory_point.path_point().theta(),
              adc_curr_info.adc_cur_heading_);
      path_point->set_theta(relative_theta);

      path_point->set_s(obstacle_trajectory_point.path_point().s());
      path_point->set_lane_id(obstacle_trajectory_point.path_point().lane_id());

      trajectory_point->set_v(obstacle_trajectory_point.v());
      trajectory_point->set_a(obstacle_trajectory_point.a());
      trajectory_point->set_relative_time(
          obstacle_trajectory_point.relative_time());
      trajectory_point->mutable_gaussian_info()->CopyFrom(
          obstacle_trajectory_point.gaussian_info());
    }
  }
}

void FeatureGenerator::GenerateObstacleFeature(
    LearningDataFrame* learning_data_frame) {
  ADCCurrentInfo adc_curr_info;
  GetADCCurrentInfo(&adc_curr_info);

  for (const auto& m : prediction_obstacles_map_) {
    auto obstacle_feature = learning_data_frame->add_obstacle();

    const auto& perception_obstale = m.second.perception_obstacle();
    obstacle_feature->set_id(m.first);
    obstacle_feature->set_length(perception_obstale.length());
    obstacle_feature->set_width(perception_obstale.width());
    obstacle_feature->set_height(perception_obstale.height());
    obstacle_feature->set_type(perception_obstale.type());

    // obstacle history trajectory points
    GenerateObstacleTrajectoryPoint(m.first, adc_curr_info, obstacle_feature);

    // obstacle prediction
    GenerateObstaclePrediction(m.second, adc_curr_info, obstacle_feature);
  }
}

void FeatureGenerator::GenerateRoutingFeature(
    const int routing_index,
    LearningDataFrame* learning_data_frame) {
  auto routing = learning_data_frame->mutable_routing();
  routing->Clear();
  for (const auto& lane_segment : routing_lane_segment_) {
    routing->add_routing_lane_id(lane_segment.first);
  }

  if (routing_index < 0) {
    return;
  }

  constexpr double kLocalRoutingLength = 200.0;
  std::vector<std::string> local_routing_lane_ids;
  // local routing land_ids behind ADS
  int i = routing_index;
  double length = 0.0;
  while (i >= 0 && length < kLocalRoutingLength) {
      local_routing_lane_ids.insert(local_routing_lane_ids.begin(),
                                    routing_lane_segment_[i].first);
      length += routing_lane_segment_[i].second;
      i--;
  }
  // local routing lane_ids ahead of ADC
  i = routing_index;
  length = 0.0;
  while (i < static_cast<int>(routing_lane_segment_.size()) &&
      length < kLocalRoutingLength) {
    local_routing_lane_ids.push_back(routing_lane_segment_[i].first);
    length += routing_lane_segment_[i].second;
    i++;
  }

  for (const auto& lane_id : local_routing_lane_ids) {
    routing->add_local_routing_lane_id(lane_id);
  }
}

void FeatureGenerator::GenerateADCTrajectoryPoints(
    const std::list<LocalizationEstimate>& localization_for_label,
    LearningDataFrame* learning_data_frame) {
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

  int i = -1;
  const int localization_sample_interval_for_trajectory_point =
      FLAGS_localization_freq / FLAGS_planning_freq;

  // use a vector to help reverse traverse list of mutable field
  std::vector<LocalizationEstimate> localization_points;
  for (const auto& le : localization_for_label) {
    ++i;
    if ((i % localization_sample_interval_for_trajectory_point) != 0) {
      continue;
    }
    localization_points.insert(localization_points.begin(), le);
  }

  int trajectory_point_index = 0;
  for (const auto& localization_point : localization_points) {
     auto adc_trajectory_point =
         learning_data_frame->add_adc_trajectory_point();
     adc_trajectory_point->set_timestamp_sec(
         localization_point.measurement_time());

    auto trajectory_point = adc_trajectory_point->mutable_trajectory_point();
    auto& pose = localization_point.pose();
    trajectory_point->mutable_path_point()->set_x(pose.position().x());
    trajectory_point->mutable_path_point()->set_y(pose.position().y());
    trajectory_point->mutable_path_point()->set_z(pose.position().z());
    trajectory_point->mutable_path_point()->set_theta(pose.heading());
    auto v = std::sqrt(pose.linear_velocity().x() * pose.linear_velocity().x() +
                       pose.linear_velocity().y() * pose.linear_velocity().y());
    trajectory_point->set_v(v);
    auto a = std::sqrt(
        pose.linear_acceleration().x() * pose.linear_acceleration().x() +
        pose.linear_acceleration().y() * pose.linear_acceleration().y());
    trajectory_point->set_a(a);

    auto planning_tag = adc_trajectory_point->mutable_planning_tag();

    // planning_tag: lane_turn
    const auto& cur_point = common::util::PointFactory::ToPointENU(
        pose.position().x(),
        pose.position().y(),
        pose.position().z());
    int routing_index;
    LaneInfoConstPtr lane = GetLane(cur_point, &routing_index);
    // lane_turn
    apollo::hdmap::Lane::LaneTurn lane_turn = apollo::hdmap::Lane::NO_TURN;
    if (lane != nullptr) {
      lane_turn = lane->lane().turn();
    }
    planning_tag->set_lane_turn(lane_turn);


    if (!FLAGS_enable_overlap_tag) {
      continue;
    }

    // planning_tag: overlap tags
    double point_distance = 0.0;
    if (trajectory_point_index > 0) {
      auto& next_point =
          learning_data_frame->adc_trajectory_point(trajectory_point_index-1)
                               .trajectory_point().path_point();
      point_distance = common::util::DistanceXY(next_point, cur_point);
    }

    common::PointENU hdmap_point;
    hdmap_point.set_x(cur_point.x());
    hdmap_point.set_y(cur_point.y());

    // clear area
    planning_tag->clear_clear_area();
    std::vector<ClearAreaInfoConstPtr> clear_areas;
    if (HDMapUtil::BaseMap().GetClearAreas(hdmap_point,
                                           kSearchRadius,
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
    if (HDMapUtil::BaseMap().GetCrosswalks(hdmap_point,
                                           kSearchRadius,
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
    if (HDMapUtil::BaseMap().GetPNCJunctions(hdmap_point,
                                           kSearchRadius,
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
      planning_tag->mutable_pnc_junction()->set_distance(pnc_junction_distance);
    }

    // signal
    std::vector<SignalInfoConstPtr> signals;
    if (HDMapUtil::BaseMap().GetSignals(hdmap_point,
                                        kSearchRadius,
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
    if (HDMapUtil::BaseMap().GetStopSigns(hdmap_point,
                                          kSearchRadius,
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
    if (HDMapUtil::BaseMap().GetYieldSigns(hdmap_point,
                                         kSearchRadius,
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

    ++trajectory_point_index;
  }

  // planning_tag
  if (learning_data_frame->adc_trajectory_point_size() >0) {
    learning_data_frame->mutable_planning_tag()->set_lane_turn(
        learning_data_frame->adc_trajectory_point(0).planning_tag()
                                                    .lane_turn());
  }
  // AINFO << "number of ADC trajectory points in one frame: "
  //       << trajectory_point_index;
}

void FeatureGenerator::GenerateLearningDataFrame() {
  int routing_index;
  LaneInfoConstPtr cur_lane = GetADCCurrentLane(&routing_index);

  auto learning_data_frame = learning_data_.add_learning_data();
  // add timestamp_sec & frame_num
  learning_data_frame->set_timestamp_sec(
      localization_for_label_.back().header().timestamp_sec());
  learning_data_frame->set_frame_num(total_learning_data_frame_num_++);

  // map_name
  learning_data_frame->set_map_name(map_name_);

  // add chassis
  auto chassis = learning_data_frame->mutable_chassis();
  chassis->CopyFrom(chassis_feature_);

  // add localization
  auto localization = learning_data_frame->mutable_localization();
  const auto& pose = localization_for_label_.back().pose();
  localization->mutable_position()->CopyFrom(pose.position());
  localization->set_heading(pose.heading());
  localization->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
  localization->mutable_linear_acceleration()->CopyFrom(
      pose.linear_acceleration());
  localization->mutable_angular_velocity()->CopyFrom(pose.angular_velocity());

  // add traffic_light
  learning_data_frame->clear_traffic_light();
  for (const auto& tl : traffic_lights_) {
    auto traffic_light = learning_data_frame->add_traffic_light();
    traffic_light->set_id(tl.first);
    traffic_light->set_color(tl.second);
  }

  // add routing
  GenerateRoutingFeature(routing_index, learning_data_frame);

  // add obstacle
  GenerateObstacleFeature(learning_data_frame);

  // add trajectory_points
  GenerateADCTrajectoryPoints(localization_for_label_, learning_data_frame);
}

void FeatureGenerator::ProcessOfflineData(const std::string& record_filename) {
  record_file_name_ =
      record_filename.substr(record_filename.find_last_of("/") + 1);
  RecordReader reader(record_filename);
  if (!reader.IsValid()) {
    AERROR << "Fail to open " << record_filename;
    return;
  }

  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == FLAGS_chassis_topic) {
      Chassis chassis;
      if (chassis.ParseFromString(message.content)) {
        OnChassis(chassis);
      }
    } else if (message.channel_name == FLAGS_localization_topic) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        OnLocalization(localization);
      }
    } else if (message.channel_name == FLAGS_hmi_status_topic) {
      HMIStatus hmi_status;
      if (hmi_status.ParseFromString(message.content)) {
        OnHMIStatus(hmi_status);
      }
    } else if (message.channel_name == FLAGS_prediction_topic) {
      PredictionObstacles prediction_obstacles;
      if (prediction_obstacles.ParseFromString(message.content)) {
        OnPrediction(prediction_obstacles);
      }
    } else if (message.channel_name == FLAGS_routing_response_topic) {
      RoutingResponse routing_response;
      if (routing_response.ParseFromString(message.content)) {
        OnRoutingResponse(routing_response);
      }
    } else if (message.channel_name == FLAGS_traffic_light_detection_topic) {
      TrafficLightDetection traffic_light_detection;
      if (traffic_light_detection.ParseFromString(message.content)) {
        OnTafficLightDetection(traffic_light_detection);
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo
