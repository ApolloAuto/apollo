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

#include "modules/storytelling/story_tellers/close_to_junction_teller.h"

#include <vector>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/storytelling/common/storytelling_gflags.h"
#include "modules/storytelling/frame_manager.h"

namespace apollo {
namespace storytelling {
namespace {

using apollo::hdmap::HDMapUtil;
using apollo::common::PathPoint;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::PNCJunctionInfoConstPtr;
using apollo::hdmap::SignalInfoConstPtr;

using apollo::planning::ADCTrajectory;

bool GetPNCJunction(const PathPoint& point,
                    std::string* pnc_junction_id) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<PNCJunctionInfoConstPtr> pnc_junctions;
  if (HDMapUtil::BaseMap().GetPNCJunctions(hdmap_point,
                                           FLAGS_search_radius,
                                           &pnc_junctions) == 0) {
    if (pnc_junctions.size() > 0) {
      *pnc_junction_id = pnc_junctions.front()->id().id();
      return true;
    }
  }
  return false;
}

bool GetJunction(const PathPoint& point, std::string* junction_id) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<JunctionInfoConstPtr> junctions;
  if (HDMapUtil::BaseMap().GetJunctions(hdmap_point,
                                        FLAGS_search_radius,
                                        &junctions) == 0) {
    if (junctions.size() > 0) {
      *junction_id = junctions.front()->id().id();
      return true;
    }
  }
  return false;
}

bool GetSignal(const PathPoint& point, std::string* signal_id) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<SignalInfoConstPtr> signals;
  if (HDMapUtil::BaseMap().GetSignals(hdmap_point,
                                      FLAGS_search_radius,
                                      &signals) == 0) {
    if (signals.size() > 0) {
      *signal_id = signals.front()->id().id();
      return true;
    }
  }
  return false;
}

}  // namespace

/**
 * @brief Get overlaps within search radius.
 */
void CloseToJunctionTeller::GetOverlaps(const ADCTrajectory& adc_trajectory) {
  static std::string overlapping_junction_id;

  const double s_start =
      adc_trajectory.trajectory_point(0).path_point().s();

  junction_id_.clear();
  junction_distance_ = -1;
  signal_id_.clear();
  signal_distance_ = -1;
  for (const auto& point : adc_trajectory.trajectory_point()) {
    const auto& path_point = point.path_point();
    if (path_point.s() > FLAGS_adc_trajectory_search_distance) {
      break;
    }

    // junction
    if (junction_id_.empty() || junction_distance_ < 0) {
      std::string junction_id;
      std::string pnc_junction_id;
      const double junction = GetJunction(path_point, &junction_id);
      const double pnc_junction = GetPNCJunction(path_point, &pnc_junction_id);
      if (pnc_junction) {
        // in PNC_JUNCTION (including overlapping with JUNCTION)
        junction_id_ = pnc_junction_id;
        junction_type_ = CloseToJunction::PNC_JUNCTION;
        junction_distance_ = path_point.s() - s_start;
        overlapping_junction_id = junction ? junction_id : "";
      } else if (junction) {
        // in JUNCTION only
        if (junction_id != overlapping_junction_id) {
          // not in JUNCTION overlapping with a PNC_JUNCTION
          junction_id_ = junction_id;
          junction_type_ = CloseToJunction::JUNCTION;
          junction_distance_ = path_point.s() - s_start;
        }
      } else {
        overlapping_junction_id.clear();
      }
    }

    // signal
    if (signal_id_.empty() || signal_distance_ < 0) {
      std::string signal_id;
      if (GetSignal(path_point, &signal_id)) {
        signal_id_ = signal_id;
        signal_distance_ = path_point.s() - s_start;
      }
    }
  }
}

void CloseToJunctionTeller::Init() {
  auto* manager = FrameManager::Instance();
  manager->CreateOrGetReader<ADCTrajectory>(FLAGS_planning_trajectory_topic);
}

void CloseToJunctionTeller::Update(Stories* stories) {
  auto* manager = FrameManager::Instance();
  static auto planning_reader = manager->CreateOrGetReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic);
  const auto trajectory = planning_reader->GetLatestObserved();
  if (trajectory == nullptr || trajectory->trajectory_point().empty()) {
    AERROR << "Planning trajectory not ready.";
    return;
  }

  GetOverlaps(*trajectory);

  // CloseToJunction
  if (!junction_id_.empty() && junction_distance_ >= 0) {
    if (!stories->has_close_to_junction()) {
      AINFO << "Enter CloseToJunction story";
    }
    auto* story = stories->mutable_close_to_junction();
    story->set_id(junction_id_);
    story->set_type(junction_type_);
    story->set_distance(junction_distance_);
  } else if (stories->has_close_to_junction()) {
    AINFO << "Exit CloseToJunction story";
    stories->clear_close_to_junction();
  }

  // CloseToSignal
  if (!signal_id_.empty() && signal_distance_ >= 0) {
    if (!stories->has_close_to_signal()) {
      AINFO << "Enter CloseToSignal story";
    }
    auto* story = stories->mutable_close_to_signal();
    story->set_id(signal_id_);
    story->set_distance(signal_distance_);
  } else if (stories->has_close_to_signal()) {
    AINFO << "Exit CloseToJunction story";
    stories->clear_close_to_signal();
  }
}

}  // namespace storytelling
}  // namespace apollo
