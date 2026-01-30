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

#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/profiler/profiler.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/algorithm/geometry/convex_hull_2d.h"
#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"
#include "modules/perception/common/lidar/common/lidar_object_util.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"

#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"

#include "modules/perception/perception_plugin/lidar_detection_filter/bigmot_refine_filter/proto/bigmot_refine_filter.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class BigmotRefineFilter : public BaseObjectFilter {
public:
    BigmotRefineFilter() = default;
    virtual ~BigmotRefineFilter() = default;

    /**
     * @brief Init of BigmotRefineFilter object
     *
     * @param options object filer options
     * @return true
     * @return false
     */
    bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions()) override;

    /**
     * @brief filter
     *
     * @param options object filter options
     * @param frame lidar frame to filter
     * @return true
     * @return false
     */
    bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

    /**
     * @brief Name of BigmotRefineFilter object
     *
     * @return std::string name
     */
    std::string Name() const override {
        return "BigmotRefineFilter";
    }

private:
    void RefineBigMotObjects(LidarFrame* frame);
    void GetExpandBBox(const base::ObjectPtr object, std::vector<base::PointD>* bbox, double expand);

private:
    double filter_time_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
        apollo::perception::lidar::BigmotRefineFilter,
        apollo::perception::lidar::BaseObjectFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
