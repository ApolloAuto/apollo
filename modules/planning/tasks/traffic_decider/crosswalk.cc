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

/**
 * @file
 **/

#include "modules/planning/tasks/traffic_decider/crosswalk.h"

#include <limits>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/traffic_decider/util.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::common::util::WithinBound;
using apollo::hdmap::HDMapUtil;
using apollo::perception::PerceptionObstacle;

Crosswalk::Crosswalk(const TrafficRuleConfig& config) : TrafficRule(config) {}

bool Crosswalk::ApplyRule(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FindCrosswalks(reference_line_info)) {
    return true;
  }

  MakeDecisions(frame, reference_line_info);
  return true;
}

void Crosswalk::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  auto* path_decision = reference_line_info->path_decision();
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  std::vector<const hdmap::PathOverlap*> crosswalks_to_stop;

  for (auto crosswalk_overlap : crosswalk_overlaps_) {
    auto crosswalk_ptr = HDMapUtil::BaseMap().GetCrosswalkById(
        hdmap::MakeMapId(crosswalk_overlap->object_id));
    auto crosswalk_info = crosswalk_ptr.get();
    std::string crosswalk_id = crosswalk_info->id().id();

    // skip crosswalk if master vehicle body already passes the stop line
    double stop_line_end_s = crosswalk_overlap->end_s;
    if (adc_front_edge_s - stop_line_end_s >
        config_.crosswalk().min_pass_s_distance()) {
      ADEBUG << "skip: crosswalk_id[" << crosswalk_id << "] stop_line_end_s["
             << stop_line_end_s << "] adc_front_edge_s[" << adc_front_edge_s
             << "]. adc_front_edge passes stop_line_end_s + buffer.";
      continue;
    }

    for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
      const PerceptionObstacle& perception_obstacle =
          path_obstacle->obstacle()->Perception();
      const std::string& obstacle_id = path_obstacle->Id();
      PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);

      // check type
      if (obstacle_type != PerceptionObstacle::PEDESTRIAN &&
          obstacle_type != PerceptionObstacle::BICYCLE &&
          obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
          obstacle_type != PerceptionObstacle::UNKNOWN) {
        ADEBUG << "obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "]. skip";
        continue;
      }

      // expand crosswalk polygon
      // note: crosswalk expanded area will include sideway area
      Vec2d point(perception_obstacle.position().x(),
                  perception_obstacle.position().y());
      const Polygon2d crosswalk_poly = crosswalk_info->polygon();
      bool in_crosswalk = crosswalk_poly.IsPointIn(point);
      const Polygon2d crosswalk_exp_poly = crosswalk_poly.ExpandByDistance(
          config_.crosswalk().expand_s_distance());
      bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);

      if (!in_expanded_crosswalk) {
        ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]: not in crosswalk expanded area";
        continue;
      }

      common::SLPoint obstacle_sl_point;
      reference_line_info->reference_line().XYToSL(
          {perception_obstacle.position().x(),
           perception_obstacle.position().y()},
          &obstacle_sl_point);
      double obstacle_l_distance = std::fabs(obstacle_sl_point.l());

      const Box2d obstacle_box =
          path_obstacle->obstacle()->PerceptionBoundingBox();
      bool is_on_road =
          reference_line_info->reference_line().HasOverlap(obstacle_box);
      bool is_path_cross =
          !path_obstacle->reference_line_st_boundary().IsEmpty();

      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] crosswalk_id[" << crosswalk_id << "] obstacle_l["
             << obstacle_sl_point.l() << "] within_crosswalk_area["
             << in_crosswalk << "] within_expanded_crosswalk_area["
             << in_expanded_crosswalk << "] is_on_road[" << is_on_road
             << "] is_path_cross[" << is_path_cross << "]";

      bool stop = false;
      if (obstacle_l_distance >= config_.crosswalk().stop_loose_l_distance()) {
        // (1) when obstacle_l_distance is big enough(>= loose_l_distance),
        //     STOP only if path crosses
        if (is_path_cross) {
          stop = true;
          ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << "] type["
                 << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
                 << "]";
        }
      } else if (obstacle_l_distance <=
                 config_.crosswalk().stop_strick_l_distance()) {
        // (2) when l_distance <= strick_l_distance + on_road(not on sideway),
        //     always STOP
        // (3) when l_distance <= strick_l_distance + not on_road(on sideway),
        //     STOP only if path crosses
        if (is_on_road || is_path_cross) {
          stop = true;
          ADEBUG << "need_stop(<=11): obstacle_id[" << obstacle_id << "] type["
                 << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
                 << "]";
        }
      } else {
        // TODO(all)
        // (4) when l_distance is between loose_l and strick_l
        //     use history decision of this crosswalk to smooth unsteadiness
        stop = true;
      }

      if (!stop) {
        ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
        continue;
      }

      // stop decision
      double stop_deceleration = util::GetADCStopDeceleration(
          reference_line_info, crosswalk_overlap->start_s,
          config_.crosswalk().min_pass_s_distance());
      if (stop_deceleration < FLAGS_max_stop_deceleration) {
        crosswalks_to_stop.push_back(crosswalk_overlap);
        ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
      }
    }
  }

  for (auto crosswalk_to_stop : crosswalks_to_stop) {
    BuildStopDecision(frame, reference_line_info,
                      const_cast<hdmap::PathOverlap*>(crosswalk_to_stop));
  }
}

bool Crosswalk::FindCrosswalks(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  crosswalk_overlaps_.clear();
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps =
      reference_line_info->reference_line().map_path().crosswalk_overlaps();
  for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
    crosswalk_overlaps_.push_back(&crosswalk_overlap);
  }
  return crosswalk_overlaps_.size() > 0;
}

bool Crosswalk::BuildStopDecision(Frame* const frame,
                                  ReferenceLineInfo* const reference_line_info,
                                  hdmap::PathOverlap* const crosswalk_overlap) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(crosswalk_overlap);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), crosswalk_overlap->start_s)) {
    ADEBUG << "crosswalk [" << crosswalk_overlap->object_id
           << "] is not on reference line";
    return true;
  }

  // create virtual stop wall
  std::string virtual_obstacle_id =
      CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, crosswalk_overlap->start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
    return false;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
    return false;
  }

  // build stop decision
  const double stop_s =
      crosswalk_overlap->start_s - config_.crosswalk().stop_distance();
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_CROSSWALK);
  stop_decision->set_distance_s(-config_.crosswalk().stop_distance());
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
