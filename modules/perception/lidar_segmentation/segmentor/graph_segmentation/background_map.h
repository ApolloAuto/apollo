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

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"

namespace apollo {
namespace perception {
namespace lidar {

struct BgNode {
    int point_number = 0;
    bool is_valid = false;
    int id = -1;
    // node feature
    float mean_height = 0.0;
    float max_height = std::numeric_limits<float>::min();
    float mean_intensity = 0.0;
    std::vector<int> pc_semantic_types = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<int> pc_motion_types = {0, 0, 0};
    bool is_important = false;
};

class BackgroundMap {
public:
    BackgroundMap() = default;
    ~BackgroundMap() = default;
    /**
     * @brief Init background map
     *
     * @param width map width
     * @param height map height
     * @param resolution grid resolution
     * @param height_threshold point height threshold
     * @return true
     * @return false
     */
    bool Init(int width, int height, float resolution, float height_threshold);

    bool Init(float xmin, float xmax, float ymin, float ymax, float resolution, float height_threshold);
    /**
     * @brief Reset all data
     *
     * @param point_number point number
     * @return true
     * @return false
     */
    bool Reset(size_t point_number);
    /**
     * @brief Update map, update mask and node
     *
     * @param frame lidar frame
     * @return true
     * @return false
     */
    bool UpdateMap(LidarFrame* frame);
    /**
     * @brief Update point mask and point index
     *
     * @param frame lidar frame
     * @return true
     * @return false
     */
    bool UpdateMask(LidarFrame* frame);
    /**
     * @brief Update node feature based on points
     *
     * @param frame lidar frame
     * @return true
     * @return false
     */
    bool UpdateNodes(LidarFrame* frame);
    /**
     * @brief Get Node Coordinate based on point
     *
     * @param x the value of x coordinate
     * @param y the value of y coordinate
     * @param grid_x x coordinate of grid
     * @param grid_y y coordinate of grid
     */
    void GetNodeCoord(float x, float y, int* grid_x, int* grid_y);
    /**
     * @brief Get valid node number
     *
     * @return int
     */
    int id() {
        return valid_node_number_;
    }

public:
    // background node, size: width * height
    std::vector<BgNode> bg_nodes_;
    // node index in bg_nodes_ for each point
    std::vector<size_t> point_idx_;
    // node importance for each point
    std::vector<size_t> point_importance_;
    // non-ground / non-foreground points
    std::vector<bool> point_mask_;

private:
    size_t point_number_;
    int width_;
    int height_;
    float resolution_;
    float height_threshold_;
    size_t valid_node_number_;

    float xmin_;
    float xmax_;
    float ymin_;
    float ymax_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
