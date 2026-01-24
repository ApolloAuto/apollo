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

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/lidar_segmentation/segmentor/graph_segmentation/proto/graph_segmentation_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/graph/graph_segmentor.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/lidar_detection/detector/cnn_segmentation/spp_engine/spp_cluster_list.h"
#include "modules/perception/lidar_detection/interface/base_lidar_detector.h"
#include "modules/perception/lidar_segmentation/segmentor/graph_segmentation/background_map.h"
#include "modules/perception/lidar_segmentation/common/bg_object_builder.h"
#include "modules/perception/lidar_segmentation/common/object_split.h"

namespace apollo {
namespace perception {
namespace lidar {

class GraphSegmentation : public BaseLidarDetector {
public:
    GraphSegmentation() = default;
    ~GraphSegmentation() = default;
    /**
     * @brief Init graph segmentation
     *
     * @param options
     * @return true
     * @return false
     */
    bool Init(const LidarDetectorInitOptions& options = LidarDetectorInitOptions()) override;
    /**
     * @brief Segment pointcloud into objects
     *
     * @param options
     * @param frame Lidar frame
     * @return true
     * @return false
     */
    bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;
    /**
     * @brief Get class name
     *
     * @return std::string
     */
    std::string Name() const override {
        return "GraphSegmentation";
    }
    /**
     * @brief Graph segment
     *
     * @param frame lidar frame
     * @return true
     * @return false
     */
    bool Segment(LidarFrame* frame);
    /**
     * @brief Compute node distance
     *
     * @param n1 node one
     * @param x1 x coordinate of node one
     * @param y1 y coordinate of node one
     * @param n2 node two
     * @param x2 x coordinate of node two
     * @param y2 y coordinate of node two
     * @return float
     */
    float NodeDistance(BgNode n1, int x1, int y1, BgNode n2, int x2, int y2);
    /**
     * @brief Get the Objects From Clusters
     *
     * @param frame lidar frame
     */
    void GetObjectsFromClusters(LidarFrame* frame);

    /**
     * @brief background filter
     *
     * @param frame lidar frame
     * @return true
     * @return false
     */
    bool Split(LidarFrame* frame);

    /**
     * @brief judge if the object need to split
     *
     * @param object
     * @return true
     * @return false
     */
    bool NeedSplit(std::shared_ptr<base::Object> object);

    /**
     * @brief Set the Bacground Object Defaul Value
     *
     * @param frame
     */
    bool SetBgObjectDefaulVal(LidarFrame* frame);

private:
    int grid_width_;
    int grid_height_;
    float resolution_;
    float threshold_;
    size_t min_pt_number_;
    int search_radius_;
    float height_threshold_;
    float semantic_cost_ = 2.0;
    BackgroundMap bg_map_;
    algorithm::GraphSegmentor graph_segmentor_;
    SppClusterList clusters_;
    GraphSegmentationConfig bg_config_;
    std::vector<std::shared_ptr<base::Object>> bg_objects_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
