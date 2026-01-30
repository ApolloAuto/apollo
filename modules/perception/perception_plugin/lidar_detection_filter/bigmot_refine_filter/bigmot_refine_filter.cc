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

#include "modules/perception/perception_plugin/lidar_detection_filter/bigmot_refine_filter/bigmot_refine_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

bool BigmotRefineFilter::Init(const ObjectFilterInitOptions& options) {
    // get config
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    BigmotRefineFilterConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

    AINFO << "Init BigmotRefineFilter Successful.";
    return true;
}

bool BigmotRefineFilter::Filter(const ObjectFilterOptions& options, LidarFrame* frame) {
    // check input
    if (frame == nullptr) {
        AERROR << "Input null frame ptr.";
        return false;
    }
    if (frame->cloud == nullptr) {
        AERROR << "Input null frame cloud.";
        return false;
    }
    if (frame->cloud->size() == 0) {
        AERROR << "Input none points.";
        return false;
    }

    AINFO << "[BeforeBigMotRefine]: " << std::to_string(frame->timestamp) << " object size is "
          << frame->segmented_objects.size();

    PERF_BLOCK("strategy_filter_bigMot_refine")
    // get filter flags
    Timer timer;
    RefineBigMotObjects(frame);
    filter_time_ = timer.toc(true);
    PERF_BLOCK_END

    AINFO << "[AfterBigMotRefine]: object size is " << frame->segmented_objects.size() << " duration is "
          << filter_time_;

    return true;
}

void BigmotRefineFilter::RefineBigMotObjects(LidarFrame* frame) {
    std::vector<base::ObjectPtr>& objects = frame->segmented_objects;
    for (size_t i = 0; i < objects.size(); i++) {
        base::ObjectPtr& vehicle_obj = objects[i];
        if (!vehicle_obj) {
            ADEBUG << " objects error, skip";
            continue;
        }
        if (vehicle_obj->type == base::ObjectType::VEHICLE) {
            for (size_t j = 0; j < objects.size(); j++) {
                if (j == i) {
                    continue;
                }
                base::ObjectPtr& other_obj = objects[j];
                if (other_obj == nullptr) {
                    continue;
                }
                if (!(other_obj->type == base::ObjectType::PEDESTRIAN
                      || other_obj->type == base::ObjectType::BICYCLE)) {
                    continue;
                }
                std::vector<base::PointD> other_obj_cloud;
                GetExpandBBox(other_obj, &other_obj_cloud, 0.5);
                base::PolygonDType big_expand_poly;
                big_expand_poly.clear();
                for (auto& pp : other_obj_cloud) {
                    big_expand_poly.push_back(pp);
                }
                ACHECK(big_expand_poly.size() == 4);

                // update lidar supplement
                base::PointFCloud vehicle_cloud = vehicle_obj->lidar_supplement.cloud;
                base::PointDCloud vehicle_cloud_world = vehicle_obj->lidar_supplement.cloud_world;
                vehicle_obj->lidar_supplement.cloud.clear();
                vehicle_obj->lidar_supplement.cloud_world.clear();
                vehicle_obj->lidar_supplement.num_points_in_roi = 0;
                for (size_t k = 0; k < vehicle_cloud.size(); k++) {
                    base::PointD vehicle_cloud_pt;
                    vehicle_cloud_pt.x = vehicle_cloud[k].x;
                    vehicle_cloud_pt.y = vehicle_cloud[k].y;
                    vehicle_cloud_pt.z = vehicle_cloud[k].z;
                    if (!algorithm::IsPointXYInPolygon2DXY(vehicle_cloud_pt, big_expand_poly)
                        && (vehicle_cloud.points_label(k) != static_cast<uint8_t>(LidarPointLabel::GROUND))) {
                        vehicle_obj->lidar_supplement.cloud.push_back(vehicle_cloud[k]);
                        vehicle_obj->lidar_supplement.cloud_world.push_back(vehicle_cloud_world[k]);
                        vehicle_obj->lidar_supplement.num_points_in_roi += 1;
                    }
                }
            }
            algorithm::ConvexHull2D<base::PointFCloud, base::PolygonDType> hull;
            if (vehicle_obj->lidar_supplement.cloud.size() > 2) {
                hull.GetConvexHull(vehicle_obj->lidar_supplement.cloud, &vehicle_obj->polygon);
            }
            // re-compute shape
            ComputeObjectShapeFromPolygon(vehicle_obj, false);
        }
    }
}

void BigmotRefineFilter::GetExpandBBox(const base::ObjectPtr object, std::vector<base::PointD>* bbox, double expand) {
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

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
