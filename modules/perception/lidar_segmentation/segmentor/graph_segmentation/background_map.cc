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

#include "modules/perception/lidar_segmentation/segmentor/graph_segmentation/background_map.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lidar {

bool BackgroundMap::Init(int width, int height, float resolution, float height_threshold) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    height_threshold_ = height_threshold;
    return true;
}

bool BackgroundMap::Init(float xmin, float xmax, float ymin, float ymax, float resolution, float height_threshold) {
    resolution_ = resolution;
    xmin_ = xmin;
    xmax_ = xmax;
    ymin_ = ymin;
    ymax_ = ymax;
    height_threshold_ = height_threshold;

    // in pointcloud-cooridinate system
    width_ = static_cast<int>((ymax - ymin) * 1.0f / resolution);
    height_ = static_cast<int>((xmax - xmin) * 1.0f / resolution);
    return true;
}

bool BackgroundMap::Reset(size_t point_number) {
    // clear all data
    bg_nodes_.clear();
    point_idx_.clear();
    point_importance_.clear();
    point_mask_.clear();
    point_number_ = point_number;
    valid_node_number_ = 0;

    // reset data
    bg_nodes_.resize(width_ * height_);
    point_idx_.resize(point_number_, 0);
    point_mask_.resize(point_number_, false);
    point_importance_.resize(point_number_, false);
    return true;
}

bool BackgroundMap::UpdateMap(LidarFrame* frame) {
    // update idx and mask
    ACHECK(UpdateMask(frame));
    // update node
    ACHECK(UpdateNodes(frame));
    return true;
}

bool BackgroundMap::UpdateMask(LidarFrame* frame) {
    if (frame->secondary_indices.indices.size() == 0) {
        AINFO << "UpdateMask, secondary_indices size is 0.";
    }
    // secondary_indices
    for (auto index : frame->secondary_indices.indices) {
        if (index < 0 || static_cast<size_t>(index) >= point_number_) {
            continue;
        }
        auto pt = frame->cloud->at(index);
        // filter point beyond height_threshold_
        Eigen::Vector4d trans_point(pt.x, pt.y, pt.z, 1);
        trans_point = frame->lidar2novatel_extrinsics * trans_point;
        if (trans_point(2) >= height_threshold_) {
            continue;
        }
        // filter point use semantic_label
        if (static_cast<PointSemanticLabel>(frame->cloud->points_semantic_label(index) & 15)
            == PointSemanticLabel::CURB) {
            continue;
        }
        // filter point outside grid
        int grid_x = -1;
        int grid_y = -1;
        GetNodeCoord(pt.x, pt.y, &grid_x, &grid_y);
        if (grid_y < 0 || grid_y >= width_ || grid_x < 0 || grid_x >= height_) {
            continue;
        }
        // valid point
        size_t grid_idx = grid_x * width_ + grid_y;
        point_mask_.at(index) = true;
        point_idx_.at(index) = grid_idx;

        // in novatel-cooridinate system
        if (trans_point(1) > FLAGS_y_back && trans_point(1) < FLAGS_y_front && trans_point(0) > FLAGS_x_back
            && trans_point(0) < FLAGS_x_front) {
            point_importance_.at(index) = true;
        }
    }

    return true;
}

bool BackgroundMap::UpdateNodes(LidarFrame* frame) {
    auto original_cloud = frame->cloud;
    for (size_t i = 0; i < point_mask_.size(); ++i) {
        if (!point_mask_.at(i)) {
            continue;
        }
        auto pt = original_cloud->at(i);
        // update node state
        size_t grid_idx = point_idx_.at(i);
        bg_nodes_.at(grid_idx).is_valid = true;
        bg_nodes_.at(grid_idx).is_important = bg_nodes_.at(grid_idx).is_important || point_importance_.at(i);

        if (bg_nodes_.at(grid_idx).point_number == 0) {
            bg_nodes_.at(grid_idx).id = valid_node_number_;
            valid_node_number_ += 1;
        }
        bg_nodes_.at(grid_idx).point_number += 1;
        // update node feature
        int pt_number = bg_nodes_.at(grid_idx).point_number;
        // mean height, mean intensity
        if (pt_number == 1) {
            bg_nodes_.at(grid_idx).mean_height = pt.z;
            bg_nodes_.at(grid_idx).mean_intensity = pt.intensity;
        } else {
            bg_nodes_.at(grid_idx).mean_height = bg_nodes_.at(grid_idx).mean_height
                    + (pt.z - bg_nodes_.at(grid_idx).mean_height) / static_cast<float>(pt_number);
            bg_nodes_.at(grid_idx).mean_intensity = bg_nodes_.at(grid_idx).mean_intensity
                    + (pt.intensity - bg_nodes_.at(grid_idx).mean_intensity) / static_cast<float>(pt_number);
        }
        // max height
        bg_nodes_.at(grid_idx).max_height = std::max(bg_nodes_.at(grid_idx).max_height, pt.z);

        // get semantic label
        PointSemanticLabel label = GetSemanticLabel(original_cloud->points_semantic_label(i));
        int index = static_cast<int>(label);
        if (index >= 0 && index < static_cast<int>(PointSemanticLabel::MAX_LABEL)) {
            bg_nodes_.at(grid_idx).pc_semantic_types[index]++;
        }
        // get motion label
        PointMotionLabel motion_label = GetMotionLabel(original_cloud->points_semantic_label(i));
        int motion_index = static_cast<int>(motion_label);
        if (motion_index >= 0 && motion_index < static_cast<int>(PointMotionLabel::MAX_LABEL)) {
            bg_nodes_.at(grid_idx).pc_motion_types[motion_index]++;
        }
    }
    return true;
}

void BackgroundMap::GetNodeCoord(float x, float y, int* grid_x, int* grid_y) {
    *grid_x = static_cast<int>((x - xmin_) / resolution_);
    *grid_y = static_cast<int>((y - ymin_) / resolution_);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
