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

#include <string>
#include <vector>
#include <limits>

#include "cyber/common/log.h"
#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"

#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"
#include "modules/perception/perception_plugin/lidar_detection_filter/mark_area_filter/proto/mark_area_filter.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class MarkAreaFilter : public BaseObjectFilter {
public:
    MarkAreaFilter() = default;
    virtual ~MarkAreaFilter() = default;

    /**
     * @brief Init of MarkAreaFilter object
     *
     * @param options object filer options
     * @return true
     * @return false
     */
    bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions()) override;

    /**
     * @brief filter mark area objects
     *
     * @param options object filter options
     * @param frame lidar frame to filter
     * @return true
     * @return false
     */
    bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override;

    /**
     * @brief Name of MarkAreaFilter object
     *
     * @return std::string name
     */
    std::string Name() const override {
        return "MarkAreaFilter";
    }

private:
    bool FilterAreaObstacles(LidarFrame* frame);
    bool ComputeAreaDistance(LidarFrame* frame);
    void FilterAllObstacles(LidarFrame* frame, std::shared_ptr<base::PolygonDType> area_polygen);
    void FilterBackgroundObstacles(LidarFrame* frame, std::shared_ptr<base::PolygonDType> area_polygen);
    void FilterForegroundObstacles(
            LidarFrame* frame,
            std::shared_ptr<base::PolygonDType> area_polygen,
            std::string area_id);
    bool IsPointInPolygon(
            LidarFrame* frame,
            std::shared_ptr<base::Object> obj,
            std::shared_ptr<base::PolygonDType> area_polygen);

private:
    MarkAreaFilterConfig mark_area_filter_config_;

    std::map<std::string, std::shared_ptr<base::PolygonDType>> mark_areas_;
    std::map<std::string, bool> area_distance_flags_;
    std::map<std::string, std::vector<base::ObjectType>> area_foreground_type_;
    std::map<std::string, int> mark_areas_type_;
    std::vector<bool> filter_flag_;

    float filter_ratio_ = 30.0;

};  // class MarkAreaFilter

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::perception::lidar::MarkAreaFilter, BaseObjectFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
