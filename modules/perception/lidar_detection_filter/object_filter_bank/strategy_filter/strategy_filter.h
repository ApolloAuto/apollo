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

#include <string>
#include <vector>
#include <limits>

#include "cyber/common/log.h"
#include "cyber/common/file.h"

#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"
#include "modules/perception/lidar_detection_filter/object_filter_bank/proto/strategy_filter_config.pb.h"

#include "modules/perception/common/algorithm/geometry/convex_hull_2d.h"
#include "modules/perception/common/lidar/common/lidar_object_util.h"

namespace apollo {
namespace perception {
namespace lidar {

class StrategyFilter : public BaseObjectFilter {
 public:
    StrategyFilter() = default;
    virtual ~StrategyFilter() = default;

    bool Init(const ObjectFilterInitOptions& options =
            ObjectFilterInitOptions()) override;

    bool Filter(const ObjectFilterOptions& options,
            LidarFrame* frame) override;

    std::string Name() const override { return "StrategyFilter"; }

 private:
    void GetExpandBBox(const base::ObjectPtr object,
        std::vector<base::PointD>* bbox, double expand) {
        bbox->clear();
        bbox->resize(4);
        Eigen::Vector3d dir = object->direction.cast<double>();
        Eigen::Vector3d odir(-dir(1), dir(0), 0.0);
        double half_l = object->size(0) * 0.5 + expand;
        double half_w = object->size(1) * 0.5 + expand;
        double center_x = object->center(0);
        double center_y = object->center(1);
        bbox->at(0).x = half_l * dir(0) + half_w * odir(0) + center_x;
        bbox->at(0).y = half_l * dir(1) + half_w * odir(1) + center_y;
        bbox->at(0).z = 0.0;
        bbox->at(1).x = -half_l * dir(0) + half_w * odir(0) + center_x;
        bbox->at(1).y = -half_l * dir(1) + half_w * odir(1) + center_y;
        bbox->at(1).z = 0.0;
        bbox->at(2).x = -half_l * dir(0) - half_w * odir(0) + center_x;
        bbox->at(2).y = -half_l * dir(1) - half_w * odir(1) + center_y;
        bbox->at(2).z = 0.0;
        bbox->at(3).x = half_l * dir(0) - half_w * odir(0) + center_x;
        bbox->at(3).y = half_l * dir(1) - half_w * odir(1) + center_y;
        bbox->at(3).z = 0.0;
    }

    void MergeInclusiveObjects(LidarFrame* frame);

    void FilterBelowGroundObjects(LidarFrame* frame);

    void FilterSmallSizeObjects(LidarFrame* frame);

 private:
    bool is_merge_inclusive_ = false;
    bool is_filter_below_objects_ = false;
    bool is_filter_small_size_ = false;
    float expand_dist_ = 0.2;
    double merge_time_ = 0.0;
    bool allow_fore_merge_ = false;
    float below_threshold_ = 0.5;
    float below_range_ = 3.0;
    float small_size_threshold_ = 0.01;
};  // class StrategyFilter

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::perception::lidar::StrategyFilter, BaseObjectFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
