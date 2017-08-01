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

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Vec2d;
using apollo::common::adapter::AdapterManager;
using apollo::common::VehicleState;

DataCenter::DataCenter() {
  _object_table.reset(new ObjectTable());

  AINFO << "Data Center is ready!";
  pnc_map_.reset(new hdmap::PncMap(FLAGS_map_filename));
}

Frame *DataCenter::frame(const uint32_t sequence_num) const {
  std::unordered_map<uint32_t, std::unique_ptr<Frame>>::const_iterator it =
      _frames.find(sequence_num);
  if (it != _frames.end()) {
    return it->second.get();
  }
  return nullptr;
}

bool DataCenter::init_current_frame(const uint32_t sequence_num) {
  if (AdapterManager::GetRoutingResult()->Empty()) {
    AERROR << "Routing is empty";
    return false;
  }
  _frame.reset(new Frame(sequence_num, pnc_map_.get()));
  common::TrajectoryPoint point;
  _frame->SetInitPose(VehicleState::instance()->pose());
  _frame->SetRouting(AdapterManager::GetRoutingResult()->GetLatestObserved());

  if (FLAGS_enable_prediction && !AdapterManager::GetPrediction()->Empty()) {
    // prediction
    const auto &prediction =
        AdapterManager::GetPrediction()->GetLatestObserved();
    ADEBUG << "Get prediction:" << prediction.DebugString();
    _frame->SetDecisionDataFromPrediction(prediction);
  }

  if (!_frame->Init()) {
    AERROR << "failed to init frame";
    return false;
  }
  return true;
}

Frame *DataCenter::current_frame() const { return _frame.get(); }

void DataCenter::save_frame() {
  _sequence_queue.push_back(_frame->sequence_num());
  _frames[_frame->sequence_num()] = std::move(_frame);
  if (_sequence_queue.size() >
      static_cast<std::size_t>(FLAGS_max_history_result)) {
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

ObjectTable *DataCenter::mutable_object_table() const {
  return _object_table.get();
}

const ObjectTable &DataCenter::object_table() const {
  return *(_object_table.get());
}

}  // namespace planning
}  // namespace apollo
