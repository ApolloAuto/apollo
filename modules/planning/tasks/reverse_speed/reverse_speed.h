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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include "modules/planning/tasks/reverse_speed/proto/reverse_speed.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

class ReverseSpeed : public Task {
public:
    bool Init(
            const std::string &config_dir,
            const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;

    apollo::common::Status Execute(Frame *frame, ReferenceLineInfo *reference_line_info) override;

private:
    apollo::common::Status Process(
            const ReferenceLineInfo *reference_line_info,
            const PathData &path_data,
            PathDecision *const path_decision);
    void GetSTboundaries(const DiscretizedPath &path_data, Frame *frame, std::vector<const STBoundary *> *boundaries);
    bool CheckOverlap(
            const common::PathPoint &path_point,
            const common::math::Polygon2d &obs_polygon,
            const double l_buffer);

private:
    ReverseSpeedConfig config_;
    common::VehicleParam vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ReverseSpeed, Task)

}  // namespace planning
}  // namespace apollo
