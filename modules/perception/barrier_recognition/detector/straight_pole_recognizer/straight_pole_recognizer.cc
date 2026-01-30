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

#include "modules/perception/barrier_recognition/detector/straight_pole_recognizer/straight_pole_recognizer.h"

#include <cmath>

#include "cyber/profiler/profiler.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/algorithm/i_lib/geometry/i_line.h"

namespace apollo {
namespace perception {
namespace lidar {

bool StraightPoleRecognizer::Init(const BarrierRecognizerInitOptions& options) {
    // get config
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    ACHECK(cyber::common::GetProtoFromFile(config_file, &barrier_config_));
    return true;
}

bool StraightPoleRecognizer::Recognize(
        const BarrierRecognizerOptions& options,
        LidarFrame* frame,
        float& open_percent) {
    std::vector<double> world_box_corner = options.world_roi_polygon;
    if (8 != world_box_corner.size()) {
        AERROR << "Incorrect barrier polygon size:" << world_box_corner.size();
        return false;
    }

    std::vector<double> extended_world_box_corner;
    ExtendBox(world_box_corner, extended_world_box_corner);

    std::vector<float> local_box_corner = {0., 0., 0., 0., 0., 0., 0., 0.};
    std::vector<base::PointF> roi_points;
    std::vector<base::PointD> roi_world_points;

    if (!Preprocess(frame, extended_world_box_corner, roi_points, 
                    roi_world_points, local_box_corner)) {
        AERROR << "Perception accurate dock preprocess failed.";
        return false;
    }

    if (roi_points.size() < barrier_config_.min_points()) {
        open_percent = 1.0;
        return true;
    }

    float anchor_x = (local_box_corner[4] + local_box_corner[6]) / 2;
    float anchor_y = (local_box_corner[5] + local_box_corner[7]) / 2;
    float line_a = local_box_corner[7] - local_box_corner[5];  // y1 - y2
    float line_b = local_box_corner[4] - local_box_corner[6];  // x2 - x1
    float line_c = local_box_corner[6] * local_box_corner[5] - 
                   local_box_corner[4] * local_box_corner[7];
    float ab_sqr_sum = line_a * line_a + line_b * line_b;
    std::vector<float> pc_samples(static_cast<int>(roi_points.size() * 2), 0);
    for (size_t i = 0; i < roi_points.size(); i++) {
        auto& pt = roi_points[i];
        float dist = line_a * pt.x + line_b * pt.y + line_c;
        float x = pt.x - line_a * dist / ab_sqr_sum;
        float y = pt.y - line_b * dist / ab_sqr_sum;
        pc_samples[i * 2] = std::sqrt((x - anchor_x) * (x - anchor_x) + 
                                      (y - anchor_y) * (y - anchor_y));
        if (x < anchor_x) {
            pc_samples[i * 2] = -pc_samples[i * 2];
        }
        pc_samples[i * 2 + 1] = pt.z;
    }
    float fit_line[3] = {0};
    algorithm::ILineFit2dTotalLeastSquare(pc_samples.data(), fit_line, roi_points.size());
    float theta = std::atan(-fit_line[0] / (fit_line[1] + 1e-6));  //  -PI/2 ~ PI/2
    theta = std::abs(theta);
    float angle = theta / M_PI * 180;
    open_percent = angle / 90;

    return true;
}

void StraightPoleRecognizer::ExtendBox(
        const std::vector<double>& box_corner,
        std::vector<double>& extended_box_corner) {
    double length = std::sqrt(std::pow(box_corner[4] - box_corner[6], 2) + 
                              std::pow(box_corner[5] - box_corner[7], 2));
    double width = std::sqrt(std::pow(box_corner[2] - box_corner[4], 2) + 
                             std::pow(box_corner[3] - box_corner[5], 2));
    extended_box_corner.resize(8);

    double length_ratio = barrier_config_.extend_dist_l() / length;
    extended_box_corner[0] = length_ratio * (box_corner[0] - box_corner[2]) + box_corner[0];
    extended_box_corner[1] = length_ratio * (box_corner[1] - box_corner[3]) + box_corner[1];
    extended_box_corner[2] = length_ratio * (box_corner[2] - box_corner[0]) + box_corner[2];
    extended_box_corner[3] = length_ratio * (box_corner[3] - box_corner[1]) + box_corner[3];
    extended_box_corner[4] = length_ratio * (box_corner[4] - box_corner[6]) + box_corner[4];
    extended_box_corner[5] = length_ratio * (box_corner[5] - box_corner[7]) + box_corner[5];
    extended_box_corner[6] = length_ratio * (box_corner[6] - box_corner[4]) + box_corner[6];
    extended_box_corner[7] = length_ratio * (box_corner[7] - box_corner[5]) + box_corner[7];

    double width_ratio = barrier_config_.extend_dist_w() / width;
    extended_box_corner[0] += width_ratio * (box_corner[0] - box_corner[6]);
    extended_box_corner[1] += width_ratio * (box_corner[1] - box_corner[7]);
    extended_box_corner[2] += width_ratio * (box_corner[2] - box_corner[4]);
    extended_box_corner[3] += width_ratio * (box_corner[3] - box_corner[5]);
    extended_box_corner[4] += width_ratio * (box_corner[4] - box_corner[2]);
    extended_box_corner[5] += width_ratio * (box_corner[5] - box_corner[3]);
    extended_box_corner[6] += width_ratio * (box_corner[6] - box_corner[0]);
    extended_box_corner[7] += width_ratio * (box_corner[7] - box_corner[1]);

    for (size_t i = 1; i < 8; i += 2) {
        extended_box_corner[i] += barrier_config_.y_offset();
    }

    for (size_t i = 0; i < 8; i += 2) {
        extended_box_corner[i] += barrier_config_.x_offset();
    }
}

float StraightPoleRecognizer::AngleBetweenTwoPoints(base::PointF& pt1, base::PointF& pt2) {
    float xy_dist = std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
    float z_dist = std::abs(pt1.z - pt2.z);
    float theta = std::atan(z_dist / xy_dist);  // 0 ~ PI/2
    float angle = theta / M_PI * 180;
    return angle;
}

bool StraightPoleRecognizer::Preprocess(
        lidar::LidarFrame* frame,
        const std::vector<double>& box_corner,
        std::vector<base::PointF>& roi_cloud,
        std::vector<base::PointD>& roi_world_cloud,
        std::vector<float>& local_box_corner) {
    // box_corner: lidar_coord
    // [left_up_x, left_up_y, right_up_x, right_up_y,
    //  right_down_x, right_down_y, left_down_x, left_down_y]
    if (frame == nullptr || frame->cloud == nullptr || frame->cloud->size() == 0) {
        AERROR << "Input no point cloud.";
        return false;
    }
    const Eigen::Affine3d& vel_pose = frame->lidar2world_pose;
    Eigen::Vector3d vel_location = vel_pose.translation();

    for (size_t i = 0; i < local_box_corner.size(); i += 2) {
        local_box_corner[i] = box_corner[i] - vel_location.x();
        local_box_corner[i + 1] = box_corner[i + 1] - vel_location.y();
    }

    std::vector<double> box_rectangular(4);
    box_rectangular[0] = std::min(std::min(std::min(box_corner[0], box_corner[2]), box_corner[4]), box_corner[6]);
    box_rectangular[1] = std::min(std::min(std::min(box_corner[1], box_corner[3]), box_corner[5]), box_corner[7]);
    box_rectangular[2] = std::max(std::max(std::max(box_corner[0], box_corner[2]), box_corner[4]), box_corner[6]);
    box_rectangular[3] = std::max(std::max(std::max(box_corner[1], box_corner[3]), box_corner[5]), box_corner[7]);

    auto& original_cloud_ = frame->cloud;
    auto& original_world_cloud_ = frame->world_cloud;
    std::vector<int> valid_point_indices;

    size_t max_point_num = 10000;
    if (barrier_config_.use_foreground_filter()) {
        max_point_num = frame->secondary_indices.indices.size();
    } else {
        max_point_num = frame->non_ground_indices.indices.size();
    }
    for (auto i = 0; i < max_point_num; ++i) {
        auto point_idx = i;
        if (barrier_config_.use_foreground_filter()) {
            point_idx = frame->secondary_indices.indices[i];
        } else {
            point_idx = frame->non_ground_indices.indices[i];
        }
        const auto world_point = original_world_cloud_->at(point_idx);
        double px = world_point.x;
        double py = world_point.y;
        float pz = original_cloud_->at(point_idx).z;
        if (px < box_rectangular[0] || px > box_rectangular[2]) {
            continue;
        }
        if (py < box_rectangular[1] || py > box_rectangular[3]) {
            continue;
        }
        double barrier_len = std::sqrt(
                                 std::pow(box_corner[4] - box_corner[6], 2) + 
                                 std::pow(box_corner[5] - box_corner[7], 2));
        float z_threshold_high = barrier_config_.z_threshold_low() + 
                                 static_cast<float>(barrier_len);
        if (pz < barrier_config_.z_threshold_low() || pz > z_threshold_high) {
            continue;
        }

        if (PtsInRoi(world_point, box_corner)) {
            original_world_cloud_->points_semantic_label(point_idx) = 7;
            base::PointF local_point;
            local_point.x = static_cast<float>(world_point.x - vel_location.x());
            local_point.y = static_cast<float>(world_point.y - vel_location.y());
            local_point.z = pz;
            roi_cloud.push_back(local_point);
            roi_world_cloud.push_back(world_point);
            valid_point_indices.push_back(point_idx);
        }
    }
    if (barrier_config_.use_foreground_filter()) {
        RecallPointsByHoverObjects(frame, box_rectangular, box_corner, 
                                   roi_cloud, roi_world_cloud);
    }
    return true;
}

bool StraightPoleRecognizer::PtsInRoi(const base::PointD& pt, 
                                      const std::vector<double>& box_corner) {
    double x1 = box_corner[0];
    double x2 = box_corner[2];
    double x3 = box_corner[4];
    double x4 = box_corner[6];
    double y1 = box_corner[1];
    double y2 = box_corner[3];
    double y3 = box_corner[5];
    double y4 = box_corner[7];

    double px = pt.x;
    double py = pt.y;

    double angle1 = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
    double angle2 = (px - x2) * (y3 - y2) - (py - y2) * (x3 - x2);
    double angle3 = (px - x3) * (y4 - y3) - (py - y3) * (x4 - x3);
    double angle4 = (px - x4) * (y1 - y4) - (py - y4) * (x1 - x4);

    if ((angle1 <= 0 && angle2 <= 0 && angle3 <= 0 && angle4 <= 0)
        || (angle1 >= 0 && angle2 >= 0 && angle3 >= 0 && angle4 >= 0)) {
        return true;
    }
    return false;
}

void StraightPoleRecognizer::RecallPointsByHoverObjects(
        const lidar::LidarFrame* frame,
        const std::vector<double>& roi_rect,
        const std::vector<double>& box_corner,
        std::vector<base::PointF>& roi_cloud,
        std::vector<base::PointD>& roi_world_cloud) {
    Eigen::Matrix<float, 3, 1> roi_center((roi_rect[0] + roi_rect[2]) / 2, 
                                          (roi_rect[1] + roi_rect[3]) / 2, 
                                          0);
    Eigen::Matrix<float, 3, 1> roi_size(roi_rect[2] - roi_rect[0], 
                                        roi_rect[3] - roi_rect[1], 
                                        1.);

    for (auto& obj : frame->segmented_objects) {
        Eigen::Matrix<float, 3, 1> obj_center = 
            (frame->lidar2world_pose * obj->center).cast<float>();
        auto iou = algorithm::CalculateIou2DXY(roi_center, roi_size, 
                                               obj_center, obj->size);
        if (iou < 0.01) {
            continue;
        }
        AINFO << "Intersection obj id:" << obj->id;
        if (HoverDetect(obj, box_corner)) {
            for (size_t j = 0; j < obj->lidar_supplement.cloud.size(); j++) {
                roi_cloud.push_back(obj->lidar_supplement.cloud.at(j));
                roi_world_cloud.push_back(obj->lidar_supplement.cloud_world.at(j));
            }
            AINFO << "Recall points of obj id:" << obj->id;
        }
    }
}

bool StraightPoleRecognizer::HoverDetect(
        const std::shared_ptr<base::Object>& object,
        const std::vector<double>& box_corner) {
    static const float invalid_ground_value = std::numeric_limits<float>::max();
    float above_min = std::numeric_limits<float>::max();
    for (size_t j = 0; j < object->lidar_supplement.cloud.size(); j++) {
        auto points_height = object->lidar_supplement.cloud.points_height(j);
        if (PtsInRoi(object->lidar_supplement.cloud_world.at(j), box_corner)) {
            above_min = std::min(above_min, points_height);
        }
    }
    return (above_min != invalid_ground_value) && (above_min > 0.5);
}

PERCEPTION_REGISTER_CURB_DETECTOR(StraightPoleRecognizer);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
