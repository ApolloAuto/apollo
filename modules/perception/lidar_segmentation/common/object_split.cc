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

#include "modules/perception/lidar_segmentation/common/object_split.h"

namespace apollo {
namespace perception {
namespace lidar {

void SplitObject(const base::ObjectPtr& obj, std::vector<base::ObjectPtr>* split_objs, float split_distance) {
    split_objs->clear();
    const auto max_edge = std::max(obj->size(0), obj->size(1));
    if (max_edge < split_distance) {
        split_objs->push_back(obj);
        return;
    }
    const auto& cloud = obj->lidar_supplement.cloud;
    const auto& cloud_world = obj->lidar_supplement.cloud_world;
    const auto& point_ids = obj->lidar_supplement.point_ids;
    Eigen::Vector3f params;
    if (!LineFit2D(obj->lidar_supplement.cloud, &params)) {
        split_objs->push_back(obj);
        return;
    }

    Eigen::Vector2f offset(cloud.at(0).x, cloud.at(0).y);
    Eigen::Vector2f min_xy(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector2f max_xy(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Eigen::Vector2f odirection = params.head(2);
    Eigen::Vector2f direction(odirection(1), -odirection(0));
    std::vector<Eigen::Vector2f> transfrom_cloud;
    transfrom_cloud.resize(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud.at(i);
        auto& vec_pt = transfrom_cloud.at(i);
        Eigen::Vector2f vec(pt.x, pt.y);
        vec -= offset;
        vec_pt(0) = direction.dot(vec);
        vec_pt(1) = odirection.dot(vec);
        min_xy(0) = std::min(min_xy(0), vec_pt(0));
        min_xy(1) = std::min(min_xy(1), vec_pt(1));
        max_xy(0) = std::max(max_xy(0), vec_pt(0));
        max_xy(1) = std::max(max_xy(1), vec_pt(1));
    }

    const auto size = max_xy - min_xy;
    size_t segments = size(0) / split_distance + 1;
    base::ObjectPool::Instance().BatchGet(segments, split_objs);
    base::Object tmp_obj = *obj;
    tmp_obj.polygon.clear();
    tmp_obj.lidar_supplement.cloud.clear();
    tmp_obj.lidar_supplement.cloud_world.clear();
    tmp_obj.lidar_supplement.point_ids.clear();
    tmp_obj.confidence = obj->confidence;
    for (auto& obj : *split_objs) {
        *obj = tmp_obj;
    }

    for (size_t i = 0; i < transfrom_cloud.size(); ++i) {
        auto& vec_pt = transfrom_cloud.at(i);
        const int section = (vec_pt(0) - min_xy(0)) / split_distance;
        auto& obj = split_objs->at(section);
        obj->lidar_supplement.cloud.push_back(cloud.at(i));
        obj->lidar_supplement.cloud_world.push_back(cloud_world.at(i));
        obj->lidar_supplement.point_ids.push_back(point_ids.at(i));
    }

    size_t valid = 0;
    for (auto& obj : *split_objs) {
        if (obj->lidar_supplement.cloud.size() > 3) {
            obj->lidar_supplement.num_points_in_roi = obj->lidar_supplement.point_ids.size();
            split_objs->at(valid++) = obj;
        }
    }
    split_objs->resize(valid);
}

bool LineFit2D(const base::PointFCloud& cloud, Eigen::Vector3f* params) {
    if (cloud.size() < 2) {
        return false;
    }
    Eigen::Vector2f mean(0, 0);
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud.at(i);
        mean(0) += pt.x;
        mean(1) += pt.y;
    }
    mean /= cloud.size();
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud.at(i);
        float dx = pt.x - mean(0);
        float dy = pt.y - mean(1);
        covariance(0, 0) += dx * dx;
        covariance(1, 1) += dy * dy;
        covariance(0, 1) += dx * dy;
    }
    covariance(1, 0) = covariance(0, 1);
    covariance /= cloud.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    Eigen::Vector2f values = solver.eigenvalues();
    Eigen::Vector2f normal
            = std::abs(values(0)) < std::abs(values(1)) ? solver.eigenvectors().col(0) : solver.eigenvectors().col(1);
    normal.normalize();
    params->head(2) = normal;
    (*params)[2] = -normal.dot(mean);
    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
