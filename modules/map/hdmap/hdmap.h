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

#ifndef MODULES_MAP_MAP_LOADER_INC_HDMAP_H
#define MODULES_MAP_MAP_LOADER_INC_HDMAP_H

#include <memory>
#include <string>
#include <vector>

#include "modules/common/macro.h"

#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_overlap.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_impl.h"

namespace apollo {
namespace hdmap {

class HDMap {
 public:
  int load_map_from_file(const std::string& map_filename);

  LaneInfoConstPtr get_lane_by_id(const apollo::hdmap::Id& id);
  JunctionInfoConstPtr get_junction_by_id(const apollo::hdmap::Id& id);
  SignalInfoConstPtr get_signal_by_id(const apollo::hdmap::Id& id);
  CrosswalkInfoConstPtr get_crosswalk_by_id(const apollo::hdmap::Id& id);
  StopSignInfoConstPtr get_stop_sign_by_id(const apollo::hdmap::Id& id);
  YieldSignInfoConstPtr get_yield_sign_by_id(const apollo::hdmap::Id& id);
  OverlapInfoConstPtr get_overlap_by_id(const apollo::hdmap::Id& id);

  int get_lanes(const apollo::hdmap::Point& point, double distance,
                std::vector<LaneInfoConstPtr>* lanes);
  int get_junctions(const apollo::hdmap::Point& point, double distance,
                    std::vector<JunctionInfoConstPtr>* junctions);
  int get_signals(const apollo::hdmap::Point& point, double distance,
                  std::vector<SignalInfoConstPtr>* signals);
  int get_crosswalks(const apollo::hdmap::Point& point, double distance,
                     std::vector<CrosswalkInfoConstPtr>* crosswalks);
  int get_stop_signs(const apollo::hdmap::Point& point, double distance,
                     std::vector<StopSignInfoConstPtr>* stop_signs);
  int get_yield_signs(const apollo::hdmap::Point& point, double distance,
                      std::vector<YieldSignInfoConstPtr>* yield_signs);

  int get_nearest_lane(const apollo::hdmap::Point& point,
                       LaneInfoConstPtr* nearest_lane, double* nearest_s,
                       double* nearest_l);

 private:
  HDMapImpl _impl;

  DISALLOW_COPY_AND_ASSIGN(HDMap);
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_MAP_LOADER_INC_HDMAP_H
