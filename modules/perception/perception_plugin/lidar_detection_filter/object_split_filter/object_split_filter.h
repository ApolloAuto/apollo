/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "Eigen/Core"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/util/eigen_defs.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_types.h"

#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"
#include "modules/perception/lidar_segmentation/common/object_split.h"
#include "modules/perception/lidar_segmentation/common/bg_object_builder.h"
#include "modules/perception/perception_plugin/lidar_detection_filter/object_split_filter/proto/object_split_filter.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class ObjectSplitFilter : public BaseObjectFilter {
public:
    ObjectSplitFilter() = default;
    virtual ~ObjectSplitFilter() = default;

    /**
     * @brief Init of ObjectSplitFilter object
     *
     * @param options object filer options
     * @return true
     * @return false
     */
    bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions()) override;

    /**
     * @brief object split
     *
     * @param options object filter options
     * @param frame lidar frame to filter
     * @return true
     * @return false
     */
    bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

    /**
     * @brief Object split filter name
     *
     * @return std::string
     */
    std::string Name() const override {
        return "ObjectSplitFilter";
    }

private:
    /**
     * @brief check if a object is cross with ego car
     *
     * @param object
     * @return true
     * @return false
     */
    bool IsCrossWithEgo(const base::ObjectPtr& object);

    /**
     * @brief check if two line segments are cross
     *
     * @param p1 line segment 1 start point
     * @param q1 line segment 1 end point
     * @param p2 line segment 2 start point
     * @param q2 line segment 2 end point
     * @return true
     * @return false
     */
    bool IsLineSegmentsCross(base::PointD p1, base::PointD q1, base::PointD p2, base::PointD q2);
    bool CheckForegroundBackground(const base::ObjectPtr& object);
    void SetSplitObjectsValue(const base::ObjectPtr& base_object, std::vector<base::ObjectPtr>* split_objects);
    int NextID();

private:
    ObjectSplitFilterConfig config_;
    std::vector<std::shared_ptr<base::Object>> split_objects_;
    int id_ = 10000;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
        apollo::perception::lidar::ObjectSplitFilter,
        apollo::perception::lidar::BaseObjectFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
