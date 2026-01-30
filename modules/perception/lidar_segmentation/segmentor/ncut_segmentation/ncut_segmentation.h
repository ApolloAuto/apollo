/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#ifdef DEBUG_NCUT
#include "pcl/visualization/pcl_visualizer.h"
#endif

#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/proto/ncut_param.pb.h"

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/lidar_detection/interface/base_lidar_detector.h"
#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/ncut.h"

namespace apollo {
namespace perception {
namespace lidar {

class NCutSegmentation : public BaseLidarDetector {
public:
    NCutSegmentation() = default;
    virtual ~NCutSegmentation(){};
    /**
     * @brief Init ncut segmentation
     *
     * @param options
     * @return true
     * @return false
     */
    bool Init(const LidarDetectorInitOptions& options = LidarDetectorInitOptions()) override;
    /**
     * @brief Detection function of ncut
     *
     * @param options
     * @param frame location to save segment objects
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
        return "NCutSegmentation";
    }
    /**
     * @brief Reset roi and ground flag false
     *
     */
    void ByPassROIService() {
        remove_roi_ = false;
        remove_ground_ = false;
    }

private:
    bool Configure(std::string model_name);

    void PartitionConnectedComponents(
            const base::PointFCloudPtr& in_cloud,
            float cell_size,
            std::vector<base::PointFCloudPtr>* out_clouds);

    void ObstacleFilter(
            const base::PointFCloudPtr& in_cloud,
            float cell_size,
            bool filter_pedestrian_only,
            base::PointFCloudPtr* out_cloud,
            std::vector<base::ObjectPtr>* segments);

    bool IsOutlier(const base::PointFCloudPtr& in_cloud);

    base::ObjectType Label2Type(const std::string& label);

    // reference pointer of lidar frame
    LidarFrame* lidar_frame_ref_ = nullptr;
    std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
    std::shared_ptr<base::AttributePointCloud<base::PointD>> original_world_cloud_;
    std::shared_ptr<base::AttributePointCloud<base::PointF>> roi_cloud_;
    std::shared_ptr<base::AttributePointCloud<base::PointD>> roi_world_cloud_;

    std::vector<std::shared_ptr<NCut>> _segmentors;
    // for outliers, must be "unknown"
    std::unique_ptr<std::vector<base::ObjectPtr>> _outliers;
    float grid_radius_ = 100.0f;
    float height_threshold_ = 2.5f;
    float partition_cell_size_ = 1.0f;
    float vehicle_filter_cell_size_ = 1.0f;
    float pedestrian_filter_cell_size_ = 0.05f;
    float outlier_length_ = 0.3f;
    float outlier_width_ = 0.3f;
    float outlier_height_ = 0.3f;
    int outlier_min_num_points_ = 10;
    bool remove_ground_ = false;
    bool remove_roi_ = false;
    bool filter_vehicle_ = false;
    bool filter_pedestrian_ = false;
    bool do_classification_ = true;
    int num_threads_ = 1;
    NCutParam ncut_param_;

#ifdef DEBUG_NCUT
    pcl::visualization::PCLVisualizer::Ptr _viewer;
    CPointCloudPtr _rgb_cloud;
    char _viewer_id[128];
    int _viewer_count;
    void VisualizePointCloud(const base::PointFCloudPtr& cloud);
    void VisualizeSegments(const std::vector<base::ObjectPtr>& segments);
    void VisualizeComponents(const base::PointFCloudPtr& cloud, const std::vector<std::vector<int>>& component_points);
#endif
};  // class NCutSegmentation

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
