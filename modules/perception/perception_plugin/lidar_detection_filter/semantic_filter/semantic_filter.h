/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cctype>
#include <numeric>

#include "modules/perception/common/util.h"
#include "cyber/plugin_manager/plugin_manager.h"

#include "modules/perception/common/base/object_types.h"
#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"
#include "modules/perception/perception_plugin/lidar_detection_filter/semantic_filter/proto/semantic_filter.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::base::kName2ObjectSemanticTypeMap;

class SemanticFilter : public BaseObjectFilter {
public:
    SemanticFilter() = default;
    virtual ~SemanticFilter() = default;

    /**
     * @brief Init of SemanticFilter object
     *
     * @param options object filer options
     * @return true
     * @return false
     */
    bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions()) override;

    /**
     * @brief filter specific semantic-type objects
     *
     * @param options object filter options
     * @param frame lidar frame to filter
     * @return true
     * @return false
     */
    bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

    /**
     * @brief Name of SemanticFilter object
     *
     * @return std::string name
     */
    std::string Name() const override {
        return "SemanticFilter";
    }

private:
    bool FillFilterFlags(LidarFrame* frame);
    bool FillFilterFlagsOfGround(LidarFrame* frame);
    bool FillFilterFlagsOfFrontCritical(LidarFrame* frame);

private:
    std::vector<bool> filter_flag_;
    std::map<base::ObjectSemanticType, int> semantic_type_map_;
    std::map<base::ObjectSemanticType, std::pair<int, float>> front_critical_filter_map_;
    float ground_prob_thr_;
    float ground_height_thr_;
    bool is_filter_cone_;
    bool only_filter_cluster_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
        apollo::perception::lidar::SemanticFilter,
        apollo::perception::lidar::BaseObjectFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
