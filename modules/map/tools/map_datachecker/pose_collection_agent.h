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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_POSE_COLLECTION_AGENT_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_POSE_COLLECTION_AGENT_H

#include <memory>
#include <mutex>
#include <vector>
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/pose_collection.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/map/tools/map_datachecker/pj_transformer.h"

namespace apollo {
namespace hdmap {

class PoseCollectionAgent {
 public:
    explicit PoseCollectionAgent(std::shared_ptr<JSonConf> sp_conf);
    using GnssBestPose_t = const apollo::drivers::gnss::GnssBestPose;
    void on_bestgnsspos_callback(
        const std::shared_ptr<GnssBestPose_t> &bestgnsspos);
    std::shared_ptr<std::vector<FramePose>> get_poses();

 private:
    void reset();

 private:
    std::mutex _mutex;
    std::shared_ptr<PoseCollection> _sp_pose_collection = nullptr;
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<PJTransformer> _sp_pj_transformer = nullptr;
};

}  // namespace hdmap
}  // namespace apollo


#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_POSE_COLLECTION_AGENT_H
