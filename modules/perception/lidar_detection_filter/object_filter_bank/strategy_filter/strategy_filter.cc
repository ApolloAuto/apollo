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

#include <algorithm>

#include "modules/perception/lidar_detection_filter/object_filter_bank/strategy_filter/strategy_filter.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"

#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace lidar {

bool StrategyFilter::Init(const ObjectFilterInitOptions& options) {
    std::string config_file =
        GetConfigFile(options.config_path, options.config_file);
    StrategyFilterConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
    is_merge_inclusive_ = config.is_merge_inclusive();
    expand_dist_ = config.expand_bbox_dist();
    allow_fore_merge_ = config.allow_fore_merge();
    is_filter_below_objects_ = config.is_filter_below_objects();
    below_threshold_ = config.below_threshold();
    below_range_ = config.below_range();
    is_filter_small_size_ = config.is_filter_small_size();
    small_size_threshold_ = config.small_size_thres();
    AINFO << "[StrategyFilter] expand_dist is " << expand_dist_;
    return true;
}

void StrategyFilter::MergeInclusiveObjects(LidarFrame* frame) {
    struct ObjectAttribute {
        size_t index = 0;
        float area = 0.0;
        float need_refine = false;
        // base::PolygonDType expand_bbox2d;
        std::vector<base::PointD> expand_bbox2d_bak;
    };

    std::vector<base::ObjectPtr>& objects = frame->segmented_objects;
    std::vector<ObjectAttribute> sorted_objects;
    sorted_objects.clear();
    sorted_objects.resize(objects.size());
    // sort_objects attribute acquire
    for (size_t i = 0; i < objects.size(); i++) {
        if (!objects.at(i)) {
            ADEBUG << " objects error, skip";
            continue;
        }
        sorted_objects[i].index = i;
        sorted_objects[i].area = objects[i]->size(0) * objects[i]->size(1);
        sorted_objects[i].need_refine = false;
        if (objects[i]->type == base::ObjectType::VEHICLE) {
            GetExpandBBox(objects[i], &sorted_objects[i].expand_bbox2d_bak,
                expand_dist_ + 0.1);
        } else {
            GetExpandBBox(objects[i], &sorted_objects[i].expand_bbox2d_bak,
                expand_dist_);
        }
    }
    // sorted by area
    auto cmp = [&](ObjectAttribute& tmp1, ObjectAttribute& tmp2) {
        if (tmp1.area == tmp2.area) {
            return tmp1.index < tmp2.index;
        }
        return tmp1.area > tmp2.area;
    };
    std::sort(sorted_objects.begin(), sorted_objects.end(), cmp);

    for (size_t i = 0; i < sorted_objects.size(); i++) {
        base::ObjectPtr& small_obj = objects[sorted_objects[i].index];
        if (small_obj == nullptr) {
            continue;
        }
        base::PointD small_obj_center;
        small_obj_center.x = small_obj->center(0);
        small_obj_center.y = small_obj->center(1);
        small_obj_center.z = small_obj->center(2);
        for (size_t j = 0; j < i; j++) {
            base::ObjectPtr& big_obj = objects[sorted_objects[j].index];
            if (big_obj == nullptr) {
                continue;
            }
            // the polygon should use expand bboxed
            std::vector<base::PointD> big_expand_vector =
                sorted_objects[j].expand_bbox2d_bak;
            // from vector to polygon
            base::PolygonDType big_expand_poly;
            big_expand_poly.clear();
            for (auto& pp : big_expand_vector) {
                big_expand_poly.push_back(pp);
            }
            ACHECK(big_expand_poly.size() == 4);

            // if (big_obj->lidar_supplement.is_background &&
            //     !small_obj->lidar_supplement.is_background) {
            //     continue;
            // }
            if (big_obj->lidar_supplement.is_background) {
                continue;
            }
            // when NOT allow merge pedestrian/bicycle/vehicle,
            // small_obj must be background-objects or trafficcone
            if (!allow_fore_merge_ && static_cast<int>(small_obj->type) >= 3) {
                continue;
            }
            if (small_obj->lidar_supplement.cloud.size() == 0 ||
                big_obj->lidar_supplement.cloud.size() == 0) {
                continue;
            }
            // small-center in big-polygon
            if (!algorithm::IsPointXYInPolygon2DXY(small_obj_center,
                big_expand_poly)) {
                continue;
            }
            ADEBUG << "BIG_OBJ: " << big_obj->id
                << " type: " << static_cast<int>(big_obj->type)
                << ", [" << big_obj->center(0) << ", "
                << big_obj->center(1) << ", " << big_obj->center(2)
                << ", " << big_obj->size(0) << ", " << big_obj->size(1) << ", "
                << big_obj->size(2) << ", " << big_obj->theta << "]; "
                << " is_background: "
                << big_obj->lidar_supplement.is_background << std::endl
                << "SMALL_OBJ: " << small_obj->id
                << " type: " << static_cast<int>(small_obj->type)
                << ", [" << small_obj->center(0)
                << ", " << small_obj->center(1) << ", " << small_obj->center(2)
                << ", " << small_obj->size(0) << ", " << small_obj->size(1)
                << ", " << small_obj->size(2) << ", " << small_obj->theta
                << "]; is_background: "
                << small_obj->lidar_supplement.is_background;
            // small-polygon in big-polygon
            bool merge_flag = true;
            for (auto& pt : small_obj->polygon) {
                if (!algorithm::IsPointXYInPolygon2DXY(pt, big_expand_poly)) {
                    merge_flag = false;
                    break;
                }
            }
            // should merge -> big completely inlcude small
            if (merge_flag) {
                big_obj->lidar_supplement.cloud +=
                    small_obj->lidar_supplement.cloud;
                big_obj->lidar_supplement.cloud_world +=
                    small_obj->lidar_supplement.cloud_world;
                big_obj->lidar_supplement.num_points_in_roi +=
                    small_obj->lidar_supplement.num_points_in_roi;
                big_obj->lidar_supplement.point_ids.insert(
                    big_obj->lidar_supplement.point_ids.end(),
                    small_obj->lidar_supplement.point_ids.begin(),
                    small_obj->lidar_supplement.point_ids.end());

                small_obj->lidar_supplement.cloud.clear();
                small_obj->lidar_supplement.cloud_world.clear();
                small_obj->lidar_supplement.point_ids.clear();

                sorted_objects[j].need_refine = true;
                AINFO << "BIG: " << std::to_string(big_obj->id)
                      << " INCLUDE SMALL: " << std::to_string(small_obj->id);
                break;
            }
        }
    }

    for (auto& obj : sorted_objects) {
        if (obj.need_refine) {
            algorithm::ConvexHull2D<base::PointFCloud, base::PolygonDType> hull;
            if (objects[obj.index]->lidar_supplement.cloud.size() > 2) {
                hull.GetConvexHull(objects[obj.index]->lidar_supplement.cloud,
                    &objects[obj.index]->polygon);
            }
            // re-compute shape
            ComputeObjectShapeFromPolygon(objects[obj.index], false);
        }
    }
    size_t valid_pos = 0;
    for (size_t i = 0; i < objects.size(); ++i) {
        if (objects[i]->lidar_supplement.cloud.size() > 0) {
            if (i != valid_pos) {
                objects[valid_pos] = objects[i];
            }
            ++valid_pos;
        }
    }
    objects.resize(valid_pos);
}

void StrategyFilter::FilterBelowGroundObjects(LidarFrame* frame) {
    std::vector<base::ObjectPtr>& objects = frame->segmented_objects;
    std::vector<bool> filter_flag(objects.size(), false);

    // filter below ground
    for (size_t i = 0; i < objects.size(); ++i) {
        float z_diff = frame->original_ground_z - objects[i]->center(2);
        float dist = sqrt(objects[i]->center(0) * objects[i]->center(0) +
            objects[i]->center(1) * objects[i]->center(1));
        // type limit
        bool type_flag = static_cast<int>(objects[i]->type) < 3;
        if (type_flag && z_diff >= below_threshold_ && dist <= below_range_) {
            AINFO << "[BelowGround] id: " << objects[i]->id
                  << " diff: " << z_diff;
            filter_flag[i] = true;
        }
    }
    size_t valid_pos = 0;
    for (size_t i = 0; i < objects.size(); ++i) {
        if (filter_flag.at(i)) {
            continue;
        }
        objects.at(valid_pos) = objects.at(i);
        ++valid_pos;
    }
    objects.resize(valid_pos);
    AINFO << "[FilterBelowGroundObjects] from " << filter_flag.size()
          << " to " << valid_pos;
}

void StrategyFilter::FilterSmallSizeObjects(LidarFrame* frame) {
    std::vector<base::ObjectPtr>& objects = frame->segmented_objects;
    std::vector<bool> filter_flag(objects.size(), false);

    // filter small size object
    for (size_t i = 0; i < objects.size(); ++i) {
        if (objects[i]->size(0) <= small_size_threshold_ &&
            objects[i]->size(1) <= small_size_threshold_) {
            AINFO << "[SmallSize] id: " << objects[i]->id << " pc size is "
                  << objects[i]->lidar_supplement.cloud.size() << " size is: "
                  << objects[i]->size(0) << ", " << objects[i]->size(0);
            filter_flag[i] = true;
        }
    }
    size_t valid_pos = 0;
    for (size_t i = 0; i < objects.size(); ++i) {
        if (filter_flag.at(i)) {
            continue;
        }
        objects.at(valid_pos) = objects.at(i);
        ++valid_pos;
    }
    objects.resize(valid_pos);
    AINFO << "[FilterSmallSizeObjects] from " << filter_flag.size()
          << " to " << valid_pos;
}

bool StrategyFilter::Filter(const ObjectFilterOptions& options,
    LidarFrame* frame) {
    if (!frame) {
        AINFO << "Lidar frame is nullptr. StrategyFilter NOT Enter";
        return false;
    }
    AINFO << "[BeforeMergeInclusive]: "
          << std::to_string(frame->timestamp) << " object size is "
          << frame->segmented_objects.size();

    if (is_filter_below_objects_) {
        FilterBelowGroundObjects(frame);
    }

    if (is_filter_small_size_) {
        FilterSmallSizeObjects(frame);
    }

    PERF_BLOCK("strategy_filter_merge")
    if (is_merge_inclusive_) {
        Timer timer;
        MergeInclusiveObjects(frame);
        merge_time_ = timer.toc(true);
    }
    PERF_BLOCK_END
    AINFO << "[AfterMergeInclusive]: object size is "
          << frame->segmented_objects.size() << " duration is " << merge_time_;
    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
