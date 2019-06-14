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
#include "modules/perception/lidar/lib/segmentation/ncut/ncut_segmentation.h"

#include <omp.h>

#include <algorithm>
#include <map>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::GetProtoFromFile;
using Eigen::MatrixXf;

bool NCutSegmentation::Init(const SegmentationInitOptions& options) {
  if (!Configure("NCutSegmentation")) {
    AERROR << "failed to load ncut config.";
    return false;
  }

  // init ground detector
  ground_detector_.reset(
      BaseGroundDetectorRegisterer::GetInstanceByName(ground_detector_str_));
  CHECK_NOTNULL(ground_detector_.get());
  GroundDetectorInitOptions ground_detector_init_options;
  CHECK(ground_detector_->Init(ground_detector_init_options))
      << "Failed to init ground detection.";

  // init roi filter
  roi_filter_.reset(
      BaseROIFilterRegisterer::GetInstanceByName(roi_filter_str_));
  CHECK_NOTNULL(roi_filter_.get());
  ROIFilterInitOptions roi_filter_init_options;
  CHECK(roi_filter_->Init(roi_filter_init_options))
      << "Failed to init roi filter.";

  _outliers.reset(new std::vector<ObjectPtr>);
  if (!_outliers) {
    AERROR << "Failed to reset outliers.";
    return false;
  }
  int num_threads = 0;
#pragma omp parallel
  { num_threads = omp_get_num_threads(); }
  _segmentors.resize(num_threads);
  for (int i = 0; i < num_threads; ++i) {
    _segmentors[i].reset(new NCut);
    if (!(_segmentors[i]->Init())) {
      AERROR << "failed to init NormalizedCut " << i << ".";
      return false;
    }
  }

  roi_cloud_ = base::PointFCloudPool::Instance().Get();
  roi_world_cloud_ = base::PointDCloudPool::Instance().Get();

  // init thread worker
  worker_.Bind([&]() {
    ROIFilterOptions roi_filter_options;
    if (lidar_frame_ref_->hdmap_struct != nullptr &&
        roi_filter_->Filter(roi_filter_options, lidar_frame_ref_)) {
      roi_cloud_->CopyPointCloud(*lidar_frame_ref_->cloud,
                                 lidar_frame_ref_->roi_indices);
      roi_world_cloud_->CopyPointCloud(*lidar_frame_ref_->world_cloud,
                                       lidar_frame_ref_->roi_indices);
    } else {
      AINFO << "Fail to call roi filter, use origin cloud.";
      lidar_frame_ref_->roi_indices.indices.resize(original_cloud_->size());
      // we manually fill roi indices with all cloud point indices
      std::iota(lidar_frame_ref_->roi_indices.indices.begin(),
                lidar_frame_ref_->roi_indices.indices.end(), 0);
      // note roi cloud's memory should be kept here
      *roi_cloud_ = *original_cloud_;
      *roi_world_cloud_ = *original_world_cloud_;
    }
    lidar_frame_ref_->cloud = roi_cloud_;
    lidar_frame_ref_->world_cloud = roi_world_cloud_;
    GroundDetectorOptions ground_detector_options;
    ground_detector_->Detect(ground_detector_options, lidar_frame_ref_);
    return true;
  });

  worker_.Start();

  AINFO << "NCutSegmentation init success, num_threads: " << num_threads;
  return true;
}

bool NCutSegmentation::Configure(std::string param_file) {
  NCutSegmentationParam ncut_param_;
  // get cnnseg params
  CHECK(GetProtoFromFile(param_file, &ncut_param_))
      << "Failed to parse CNNSegParam config file." << param_file;
  grid_radius_ = ncut_param_.grid_radius();
  height_threshold_ = ncut_param_.height_threshold();
  partition_cell_size_ = ncut_param_.partition_cell_size();
  vehicle_filter_cell_size_ = ncut_param_.vehicle_filter_cell_size();
  pedestrian_filter_cell_size_ = ncut_param_.pedestrian_filter_cell_size();
  outlier_length_ = ncut_param_.outlier_length();
  outlier_width_ = ncut_param_.outlier_width();
  outlier_height_ = ncut_param_.outlier_height();
  outlier_min_num_points_ = ncut_param_.outlier_min_num_points();
  remove_ground_ = ncut_param_.remove_ground_points();
  remove_roi_ = ncut_param_.remove_roi();
  ground_detector_str_ = ncut_param_.ground_detector();
  roi_filter_str_ = ncut_param_.roi_filter();
  return true;
}

bool NCutSegmentation::Segment(const SegmentationOptions& options,
                               LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->world_cloud == nullptr) {
    AERROR << "Input null frame world cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }
  if (frame->cloud->size() != frame->world_cloud->size()) {
    AERROR << "Cloud size and world cloud size not consistent.";
    return false;
  }

  // record input cloud and lidar frame
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  std::vector<base::ObjectPtr>* segments = &(frame->segmented_objects);
  double start_t = omp_get_wtime();
  int num_threads = 1;
#pragma omp parallel
  { num_threads = omp_get_num_threads(); }

  if (remove_roi_) {
    AINFO << "remove roi and remove ground for ncut segmentation";
    worker_.WakeUp();
    worker_.Join();
  }

  base::PointFCloudPtr cloud_above_ground(new base::PointFCloud);
  if (remove_ground_) {
    cloud_above_ground->CopyPointCloud(*lidar_frame_ref_->cloud,
                                       lidar_frame_ref_->non_ground_indices);
  }

  // filter_by_ground(cloud, non_ground_indices, &cloud_above_ground);
  ADEBUG << "filter ground, elapsed time: " << omp_get_wtime() - start_t;
  start_t = omp_get_wtime();

  // .3 filter vehicle
  base::PointFCloudPtr cloud_after_car_filter;
  ObstacleFilter(cloud_above_ground, vehicle_filter_cell_size_, false,
                 &cloud_after_car_filter, segments);
  ADEBUG << "filter vehicle, elapsed time: " << omp_get_wtime() - start_t;
  start_t = omp_get_wtime();

  // .4 filter pedestrian
  base::PointFCloudPtr cloud_after_people_filter;
  ObstacleFilter(cloud_after_car_filter, pedestrian_filter_cell_size_, true,
                 &cloud_after_people_filter, segments);

  // ADEBUG << "filter pedestrian, elapsed time: " << omp_get_wtime() - start_t;
  // start_t = omp_get_wtime();

  // .5 partition into small regions
  std::vector<base::PointFCloudPtr> cloud_components;
  PartitionConnectedComponents(cloud_after_people_filter, partition_cell_size_,
                               &cloud_components);
  ADEBUG << "partition small regions, elapsed time: "
         << omp_get_wtime() - start_t;
  start_t = omp_get_wtime();

  std::vector<bool> cloud_outlier_flag(cloud_components.size());

#pragma omp parallel
  {
#pragma omp for
    for (size_t i = 0; i < cloud_components.size(); ++i) {
      cloud_outlier_flag[i] = IsOutlier(cloud_components[i]);
    }
  }

  std::vector<int> cloud_outlier;
  std::vector<int> cloud_tbd;
  for (int i = 0; i < static_cast<int>(cloud_components.size()); ++i) {
    if (cloud_outlier_flag[i]) {
      cloud_outlier.push_back(i);
    } else {
      cloud_tbd.push_back(i);
    }
  }

  // .5.1 outlier
  for (size_t i = 0; i < cloud_outlier.size(); ++i) {
    base::PointFCloudPtr pc = cloud_components[cloud_outlier[i]];
    base::ObjectPtr obj = std::make_shared<base::Object>();
    obj->lidar_supplement.cloud = *pc;
    _outliers->push_back(obj);
  }
  ADEBUG << "filter outlier, elapsed time: " << omp_get_wtime() - start_t;
  start_t = omp_get_wtime();

  // .6 graph cut each
  std::vector<std::vector<base::PointFCloudPtr>> threads_segment_pcs(
      num_threads);
  std::vector<std::vector<std::string>> threads_segment_labels(num_threads);
  std::vector<std::vector<base::PointFCloudPtr>> threads_outlier_pcs(
      num_threads);
// .6.1 process each component in parallel
#pragma omp parallel
  {
    int tid = omp_get_thread_num();
    std::shared_ptr<NCut> my_ncut = _segmentors[tid];
    std::vector<base::PointFCloudPtr>& my_segment_pcs =
        threads_segment_pcs[tid];
    std::vector<std::string>& my_segment_labels = threads_segment_labels[tid];
    std::vector<base::PointFCloudPtr>& my_outlier_pcs =
        threads_outlier_pcs[tid];
#pragma omp for schedule(guided)
    for (size_t i = 0; i < cloud_tbd.size(); ++i) {
      my_ncut->Segment(cloud_components[cloud_tbd[i]]);
      for (int j = 0; j < my_ncut->NumSegments(); ++j) {
        base::PointFCloudPtr pc = my_ncut->GetSegmentPointCloud(j);
        std::string label = my_ncut->GetSegmentLabel(j);
        if (IsOutlier(pc)) {
          my_outlier_pcs.push_back(pc);
        } else {
          my_segment_pcs.push_back(pc);
          my_segment_labels.push_back(label);
        }
      }
    }
  }

  ADEBUG << "parallel normalized cut, elapsed time: "
         << omp_get_wtime() - start_t;
  start_t = omp_get_wtime();
  // .6.2 aggregate results
  std::vector<int> segment_offset(num_threads,
                                  static_cast<int>(segments->size()));
  for (int i = 1; i < num_threads; ++i) {
    segment_offset[i] = segment_offset[i - 1] +
                        static_cast<int>(threads_segment_pcs[i - 1].size());
  }
  int new_num_segments =
      static_cast<int>(threads_segment_pcs[num_threads - 1].size()) +
      segment_offset[num_threads - 1];
  segments->resize(new_num_segments);
#pragma omp parallel for
  for (int i = 0; i < num_threads; ++i) {
    int offset = segment_offset[i];
    for (size_t j = 0; j < threads_segment_pcs[i].size(); ++j) {
      base::ObjectPtr& obj_ptr = (*segments)[offset + j];
      obj_ptr.reset(new base::Object());
      obj_ptr->lidar_supplement.cloud = *threads_segment_pcs[i][j];
    }
  }
  std::vector<int> outlier_offset(num_threads,
                                  static_cast<int>(_outliers->size()));
  for (int i = 1; i < num_threads; ++i) {
    outlier_offset[i] = outlier_offset[i - 1] +
                        static_cast<int>(threads_outlier_pcs[i - 1].size());
  }
  int new_num_outliers =
      static_cast<int>(threads_outlier_pcs[num_threads - 1].size()) +
      outlier_offset[num_threads - 1];
  _outliers->resize(new_num_outliers);
#pragma omp parallel for
  for (int i = 0; i < num_threads; ++i) {
    int offset = outlier_offset[i];
    for (size_t j = 0; j < threads_outlier_pcs[i].size(); ++j) {
      base::ObjectPtr& obj_ptr = (*_outliers)[offset + j];
      obj_ptr.reset(new base::Object);
      obj_ptr->lidar_supplement.cloud = *threads_outlier_pcs[i][j];
    }
  }
  ADEBUG << "aggregate results, elapsed time: " << omp_get_wtime() - start_t;
  return true;
}

void NCutSegmentation::PartitionConnectedComponents(
    const base::PointFCloudPtr& in_cloud, float cell_size,
    std::vector<base::PointFCloudPtr>* out_clouds) {
  std::vector<base::PointFCloudPtr>& temp_clouds = *out_clouds;
  FloodFill FFfilter(grid_radius_, cell_size);
  std::vector<std::vector<int>> component_points;
  std::vector<int> num_cells_per_components;
  FFfilter.GetSegments(in_cloud, &component_points, &num_cells_per_components);
  temp_clouds.resize(component_points.size());
  for (size_t i = 0; i < component_points.size(); ++i) {
    temp_clouds[i] = base::PointFCloudPtr(
        new base::PointFCloud(*in_cloud, component_points[i]));
  }
}

void NCutSegmentation::ObstacleFilter(const base::PointFCloudPtr& in_cloud,
                                      float cell_size,
                                      bool filter_pedestrian_only,
                                      base::PointFCloudPtr* out_cloud,
                                      std::vector<base::ObjectPtr>* segments) {
  FloodFill FFfilter(grid_radius_, cell_size);
  std::vector<std::vector<int>> component_points;
  std::vector<int> num_cells_per_components;
  FFfilter.GetSegments(in_cloud, &component_points, &num_cells_per_components);
  const unsigned int min_num_points = 50;
  const int num_components = static_cast<int>(component_points.size());
  std::vector<std::string> component_labels(num_components, "unknown");
  int tid = 0;
  for (int i = 0; i < num_components; ++i) {
    if (component_points[i].size() > min_num_points) {
      base::PointFCloudPtr pc = base::PointFCloudPtr(
          new base::PointFCloud(*in_cloud, component_points[i]));
      std::string label =
          _segmentors[tid]->GetPcRoughLabel(pc, filter_pedestrian_only);
      ADEBUG << "before: component id: " << i << ", label: " << label;
      if (filter_pedestrian_only) {
        label =
            (label == "pedestrian" || label == "nonMot") ? label : "unknown";
      }
      ADEBUG << "after: component id: " << i << ", label: " << label;
      component_labels[i] = label;
    }
  }  // end of for

  ADEBUG << "classification done";
  std::vector<int> remaining_pids;
  std::vector<int> obstacle_components;
  for (int i = 0; i < num_components; ++i) {
    if (component_labels[i] != "unknown") {
      obstacle_components.push_back(i);
    } else {
      remaining_pids.insert(remaining_pids.begin(), component_points[i].begin(),
                            component_points[i].end());
    }
  }
  ADEBUG << "obstacle_filter: filter unknown out, obstacle_components.size = "
         << obstacle_components.size()
         << ", remaining_pids.size = " << remaining_pids.size();
  int offset = static_cast<int>(segments->size());
  segments->resize(offset + obstacle_components.size());
  for (size_t i = 0; i < obstacle_components.size(); ++i) {
    int cid = obstacle_components[i];
    ObjectPtr& object_ptr = (*segments)[offset + i];
    object_ptr.reset(new base::Object);
    object_ptr->lidar_supplement.cloud.CopyPointCloud(*in_cloud,
                                                      component_points[cid]);
  }
  *out_cloud =
      base::PointFCloudPtr(new base::PointFCloud(*in_cloud, remaining_pids));
}

bool NCutSegmentation::IsOutlier(const base::PointFCloudPtr& in_cloud) {
  size_t min_num_points = std::max(outlier_min_num_points_, 1);
  if (in_cloud->size() < min_num_points) {
    return true;
  }
  float x_max = -FLT_MAX;
  float y_max = -FLT_MAX;
  float z_max = -FLT_MAX;
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  base::PointF pt_max = (*in_cloud)[0];
  for (size_t i = 0; i < in_cloud->size(); ++i) {
    const base::PointF& pt = (*in_cloud)[i];
    x_min = std::min(x_min, pt.x);
    x_max = std::max(x_max, pt.x);
    y_min = std::min(y_min, pt.y);
    y_max = std::max(y_max, pt.y);
    z_min = std::min(z_min, pt.z);
    if (pt.z > z_max) {
      z_max = pt.z;
      pt_max = pt;
    }
  }
  float length = x_max - x_min;
  float width = y_max - y_min;
  float height = z_max - z_min;
  if (length < outlier_length_ && width < outlier_width_) {
    return true;
  }
  if (height < outlier_height_) {
    return true;
  }
  // std::pair<float, bool> dist = _ground_detector.distance_to_ground(pt_max);
  // if (dist.second && dist.first < _outlier_height) {
  //    return true;
  //}
  return false;
}

PERCEPTION_REGISTER_SEGMENTATION(NCutSegmentation);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
