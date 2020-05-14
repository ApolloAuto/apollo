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

using apollo::common::PathPoint;
using apollo::hdmap::ClearAreaInfoConstPtr;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::PNCJunctionInfoConstPtr;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::hdmap::StopSignInfoConstPtr;
using apollo::hdmap::YieldSignInfoConstPtr;

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

bool GetClearArea(const PathPoint& point, std::string* clear_area_id) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<ClearAreaInfoConstPtr> clear_areas;
  if (HDMapUtil::BaseMap().GetClearAreas(hdmap_point,
                                         FLAGS_search_radius,
                                         &clear_areas) == 0) {
    if (clear_areas.size() > 0) {
      *clear_area_id = clear_areas.front()->id().id();
      return true;
    }
  }
  return false;
}

bool GetCrosswalk(const PathPoint& point, std::string* crosswalk_id) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<CrosswalkInfoConstPtr> crosswalks;
  if (HDMapUtil::BaseMap().GetCrosswalks(hdmap_point,
                                         FLAGS_search_radius,
                                         &crosswalks) == 0) {
    if (crosswalks.size() > 0) {
      *crosswalk_id = crosswalks.front()->id().id();
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

bool GetStopSign(const PathPoint& point, std::string* stop_sign_id) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<StopSignInfoConstPtr> stop_signs;
  if (HDMapUtil::BaseMap().GetStopSigns(hdmap_point,
                                        FLAGS_search_radius,
                                        &stop_signs) == 0) {
    if (stop_signs.size() > 0) {
      *stop_sign_id = stop_signs.front()->id().id();
      return true;
    }
  }
  return false;
}

bool GetYieldSign(const PathPoint& point, std::string* yield_sign_id) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<YieldSignInfoConstPtr> yield_signs;
  if (HDMapUtil::BaseMap().GetYieldSigns(hdmap_point,
                                         FLAGS_search_radius,
                                         &yield_signs) == 0) {
    if (yield_signs.size() > 0) {
      *yield_sign_id = yield_signs.front()->id().id();
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
  clear_area_id_.clear();
  clear_area_distance_ = -1;
  crosswalk_id_.clear();
  crosswalk_distance_ = -1;
  signal_id_.clear();
  signal_distance_ = -1;
  stop_sign_id_.clear();
  stop_sign_distance_ = -1;
  yield_sign_id_.clear();
  yield_sign_distance_ = -1;
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

    // clear_area
    if (clear_area_id_.empty() || clear_area_distance_ < 0) {
      std::string clear_area_id;
      if (GetClearArea(path_point, &clear_area_id)) {
        clear_area_id_ = clear_area_id;
        clear_area_distance_ = path_point.s() - s_start;
      }
    }

    // crosswalk
    if (crosswalk_id_.empty() || crosswalk_distance_ < 0) {
      std::string crosswalk_id;
      if (GetCrosswalk(path_point, &crosswalk_id)) {
        crosswalk_id_ = crosswalk_id;
        crosswalk_distance_ = path_point.s() - s_start;
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

    // stop_sign
    if (stop_sign_id_.empty() || stop_sign_distance_ < 0) {
      std::string stop_sign_id;
      if (GetStopSign(path_point, &stop_sign_id)) {
        stop_sign_id_ = stop_sign_id;
        stop_sign_distance_ = path_point.s() - s_start;
      }
    }

    // yield_sign
    if (yield_sign_id_.empty() || yield_sign_distance_ < 0) {
      std::string yield_sign_id;
      if (GetYieldSign(path_point, &yield_sign_id)) {
        yield_sign_id_ = yield_sign_id;
        yield_sign_distance_ = path_point.s() - s_start;
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

  // CloseToClearArea
  if (!clear_area_id_.empty() && clear_area_distance_ >= 0) {
    if (!stories->has_close_to_clear_area()) {
      AINFO << "Enter CloseToClearArea story";
    }
    auto* story = stories->mutable_close_to_clear_area();
    story->set_id(clear_area_id_);
    story->set_distance(clear_area_distance_);
  } else if (stories->has_close_to_clear_area()) {
    AINFO << "Exit CloseToClearArea story";
    stories->clear_close_to_clear_area();
  }

  // CloseToCrosswalk
  if (!crosswalk_id_.empty() && crosswalk_distance_ >= 0) {
    if (!stories->has_close_to_crosswalk()) {
      AINFO << "Enter CloseToCrosswalk story";
    }
    auto* story = stories->mutable_close_to_crosswalk();
    story->set_id(crosswalk_id_);
    story->set_distance(crosswalk_distance_);
  } else if (stories->has_close_to_crosswalk()) {
    AINFO << "Exit CloseToCrosswalk story";
    stories->clear_close_to_crosswalk();
  }

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
    AINFO << "Exit CloseToSignal story";
    stories->clear_close_to_signal();
  }

  // CloseToStopSign
  if (!stop_sign_id_.empty() && stop_sign_distance_ >= 0) {
    if (!stories->has_close_to_stop_sign()) {
      AINFO << "Enter CloseToStopSign story";
    }
    auto* story = stories->mutable_close_to_stop_sign();
    story->set_id(stop_sign_id_);
    story->set_distance(stop_sign_distance_);
  } else if (stories->has_close_to_stop_sign()) {
    AINFO << "Exit CloseToStopSign story";
    stories->clear_close_to_stop_sign();
  }

  // CloseToYieldSign
  if (!yield_sign_id_.empty() && yield_sign_distance_ >= 0) {
    if (!stories->has_close_to_yield_sign()) {
      AINFO << "Enter CloseToYieldSign story";
    }
    auto* story = stories->mutable_close_to_yield_sign();
    story->set_id(yield_sign_id_);
    story->set_distance(yield_sign_distance_);
  } else if (stories->has_close_to_yield_sign()) {
    AINFO << "Exit CloseToYieldSign story";
    stories->clear_close_to_yield_sign();
  }
}

}  // namespace storytelling
}  // namespace apollo
