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
/**
 * @file utils.h
 * @brief utils for v2x_proxy
 */

#pragma once

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "cyber/cyber.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/v2x/v2x_proxy/proto_adapter/proto_adapter.h"

namespace apollo {
namespace v2x {
const char *const kUnknownJunctionId = "unknown";

class InternalData final {
 private:
  static constexpr size_t kBufferSize = 40;
  std::shared_ptr<OSLight> oslight_ = nullptr;
  std::shared_ptr<ObuLight> obu_light_ = nullptr;
  int intersection_id_ = -1;
  double change_color_timestamp_ = 0.0;
  int32_t *remaining_time_ = nullptr;
  double *msg_timestamp_ = nullptr;

 public:
  InternalData();

  virtual ~InternalData();

  void reset();

  bool TrafficLightProc(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                        double distance, ::apollo::v2x::RoadTrafficLight *msg);

  bool ProcTrafficlight(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                        const ObuLight *x2v_traffic_light,
                        const std::string &junction_id, bool flag_u_turn,
                        double distance, double check_time,
                        std::shared_ptr<OSLight> *os_light);

  bool ProcPlanningMessage(
      const ::apollo::planning::ADCTrajectory *planning_msg,
      const OSLight *last_os_light,
      std::shared_ptr<::apollo::perception::TrafficLightDetection> *res_light);
};
namespace utils {

bool FindAllRoadId(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                   const ::apollo::hdmap::LaneInfoConstPtr &start_laneinfo,
                   const ::apollo::hdmap::LaneInfoConstPtr &end_laneinfo,
                   size_t max_road_count,
                   std::unordered_set<std::string> *result_id_set);

bool CheckCarInSet(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                   const std::unordered_set<std::string> &id_set,
                   const ::apollo::hdmap::LaneInfoConstPtr &car_laneinfo,
                   size_t max_lane_count);

bool GetRsuInfo(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                const OSLocation &os_location,
                const std::set<std::string> &rsu_whitelist, double distance,
                double max_heading_difference,
                std::shared_ptr<::apollo::v2x::CarStatus> *v2x_car_status,
                std::string *out_junction_id, double *out_heading);

OSLightColor GetNextColor(OSLightColor color);

void UniqueOslight(OSLight *os_light);

}  // namespace utils

}  // namespace v2x
}  // namespace apollo
