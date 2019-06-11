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
#include "modules/map/tools/map_datachecker/pose_collection.h"

namespace apollo {
namespace hdmap {

PoseCollection::PoseCollection(std::shared_ptr<JSonConf> sp_conf) {
  _sp_conf = sp_conf;
  reset();
}

void PoseCollection::reset() {
  _sp_poses = std::make_shared<std::vector<FramePose>>();
}

void PoseCollection::collect(const FramePose& pose) {
  if (_sp_poses == nullptr) {
    _sp_poses = std::make_shared<std::vector<FramePose>>();
  }
  _sp_poses->push_back(pose);
}

std::shared_ptr<std::vector<FramePose>> PoseCollection::get_poses() {
  return _sp_poses;
}

}  // namespace hdmap
}  // namespace apollo
