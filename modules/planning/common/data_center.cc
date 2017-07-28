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
 * @file:
 **/

#include "modules/planning/common/data_center.h"

#include <fstream>
#include <utility>
#include <vector>

#include "modules/map/proto/map_id.pb.h"

#include "google/protobuf/text_format.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_smoother.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;

DataCenter::DataCenter() {
  _object_table.reset(new ObjectTable());
  _master.reset(new MasterStateMachine());

  AINFO << "Data Center is ready!";

  if (map_.load_map_from_file(FLAGS_map_filename)) {
    AFATAL << "Failed to load map: " << FLAGS_map_filename;
  }
  AINFO << "map loaded, Map file: " << FLAGS_map_filename;
}

Frame *DataCenter::frame(const uint32_t sequence_num) const {
  std::unordered_map<uint32_t, std::unique_ptr<Frame>>::const_iterator it =
      _frames.find(sequence_num);
  if (it != _frames.end()) {
    return it->second.get();
  }
  return nullptr;
}

bool DataCenter::CreateReferenceLineFromMap() {
  if (AdapterManager::GetRoutingResult()->Empty()) {
    AERROR << "Routing is empty";
    return false;
  }
  const auto &routing_result =
      AdapterManager::GetRoutingResult()->GetLatestObserved();

  std::vector<ReferencePoint> ref_points;
  common::math::Vec2d vehicle_position;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
  ReferenceLineSmoother smoother;
  if (!smoother.Init(FLAGS_reference_line_smoother_config_file)) {
    AERROR << "Failed to load file "
           << FLAGS_reference_line_smoother_config_file;
    return false;
  }

  vehicle_position.set_x(common::VehicleState::instance()->x());
  vehicle_position.set_y(common::VehicleState::instance()->y());

  for (const auto &lane : routing_result.route()) {
    auto lane_id = common::util::MakeMapId(lane.id());
    ADEBUG << "Added lane from routing:" << lane.id();
    lane_info_ptr = map_.get_lane_by_id(lane_id);
    if (!lane_info_ptr) {
      AERROR << "failed to find lane " + lane.id() + " from map ";
      return false;
    }
    const auto &points = lane_info_ptr->points();
    const auto &accumulate_s = lane_info_ptr->accumulate_s();
    const auto &headings = lane_info_ptr->headings();
    for (size_t i = 0; i < points.size(); ++i) {
      hdmap::LaneWaypoint lane_waypoint(lane_info_ptr.get(), accumulate_s[i]);
      // TODO(fanhaoyang) figure out a reason for number -2.0, 2.0
      ref_points.emplace_back(points[i], headings[i], 0.0, 0.0, lane_waypoint);
    }
  }
  if (ref_points.empty()) {
    AERROR << "Found no reference points from map";
    return false;
  }

  std::unique_ptr<ReferenceLine> reference_line(new ReferenceLine(ref_points));
  std::vector<ReferencePoint> smoothed_ref_points;
  if (!smoother.smooth(*reference_line, &smoothed_ref_points)) {
    AERROR << "Fail to smooth a reference line from map";
    return false;
  }
  ADEBUG << "smooth reference points num:" << smoothed_ref_points.size();
  _frame->mutable_planning_data()->set_reference_line(smoothed_ref_points);
  return true;
}

bool DataCenter::init_current_frame(const uint32_t sequence_num) {
  _frame.reset(new Frame(sequence_num));

  if (!CreateReferenceLineFromMap()) {
    AERROR << "failed to create reference line";
    return false;
  }
  return true;
}

Frame *DataCenter::current_frame() const { return _frame.get(); }

void DataCenter::save_frame() {
  _sequence_queue.push_back(_frame->sequence_num());
  _frames[_frame->sequence_num()] = std::move(_frame);
  if (_sequence_queue.size() > static_cast<size_t>(FLAGS_max_history_result)) {
    _frames.erase(_sequence_queue.front());
    _sequence_queue.pop_front();
  }
}

const Frame *DataCenter::last_frame() const {
  if (_sequence_queue.empty()) {
    return nullptr;
  }
  uint32_t sequence_num = _sequence_queue.back();
  return _frames.find(sequence_num)->second.get();
}

MasterStateMachine *DataCenter::mutable_master() const { return _master.get(); }

ObjectTable *DataCenter::mutable_object_table() const {
  return _object_table.get();
}

const ObjectTable &DataCenter::object_table() const {
  return *(_object_table.get());
}

}  // namespace planning
}  // namespace apollo
