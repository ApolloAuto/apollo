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

#include "modules/perception/base/object.h"
#include "modules/perception/lib/thread/thread_worker.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/interface/base_ground_detector.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/interface/base_segmentation.h"
#include "modules/perception/lidar/lib/segmentation/ncut/ncut.h"
#include "modules/perception/lidar/lib/segmentation/ncut/proto/ncut_param.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::ObjectPtr;

class NCutSegmentation : public BaseSegmentation {
 public:
  NCutSegmentation() = default;
  ~NCutSegmentation() = default;

  bool Init(const SegmentationInitOptions& options =
                SegmentationInitOptions()) override;

  bool Segment(const SegmentationOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "NCutSegmentation"; }

  void ByPassROIService() {
    remove_roi_ = false;
    remove_ground_ = false;
  }

 private:
  bool Configure(std::string model_name);

  void PartitionConnectedComponents(
      const base::PointFCloudPtr& in_cloud, float cell_size,
      std::vector<base::PointFCloudPtr>* out_clouds);

  void ObstacleFilter(const base::PointFCloudPtr& in_cloud, float cell_size,
                      bool filter_pedestrian_only,
                      base::PointFCloudPtr* out_cloud,
                      std::vector<base::ObjectPtr>* segments);

  bool IsOutlier(const base::PointFCloudPtr& in_cloud);

  bool GetConfigs(std::string* ncut_file);

  base::ObjectType Label2Type(const std::string& label);

  // ground detector for background segmentation
  std::unique_ptr<BaseGroundDetector> ground_detector_;
  // roi filter for background segmentation
  std::unique_ptr<BaseROIFilter> roi_filter_;

  // reference pointer of lidar frame
  LidarFrame* lidar_frame_ref_ = nullptr;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>>
      original_world_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> roi_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>> roi_world_cloud_;
  // thread worker
  lib::ThreadWorker worker_;

  std::vector<std::shared_ptr<NCut>> _segmentors;
  // for outliers, must be "unknown"
  std::unique_ptr<std::vector<ObjectPtr>> _outliers;
  float grid_radius_ = 100.0f;
  float height_threshold_ = 2.5f;
  float partition_cell_size_ = 1.0f;
  float vehicle_filter_cell_size_ = 1.0f;
  float pedestrian_filter_cell_size_ = 0.05f;
  float outlier_length_ = 0.3f;
  float outlier_width_ = 0.3f;
  float outlier_height_ = 0.3f;
  int outlier_min_num_points_ = 10;
  bool remove_ground_ = true;
  bool remove_roi_ = true;
  bool do_classification_ = true;
  std::string ground_detector_str_;
  std::string roi_filter_str_;
  NCutParam ncut_param_;

#ifdef DEBUG_NCUT
  pcl::visualization::PCLVisualizer::Ptr _viewer;
  CPointCloudPtr _rgb_cloud;
  char _viewer_id[128];
  int _viewer_count;
  void VisualizePointCloud(const base::PointFCloudPtr& cloud);
  void VisualizeSegments(const std::vector<base::ObjectPtr>& segments);
  void VisualizeComponents(
      const base::PointFCloudPtr& cloud,
      const std::vector<std::vector<int>>& component_points);
#endif
};  // class NCutSegmentation

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
