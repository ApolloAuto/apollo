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

#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/ncut_segmentation.h"

#include <algorithm>
#include <limits>
#include <map>

#include <omp.h>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/lidar/common/cloud_mask.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetProtoFromFile;
using base::ObjectPtr;

bool NCutSegmentation::Init(const LidarDetectorInitOptions& options) {
    std::string param_file = GetConfigFile(options.config_path, options.config_file);
    AINFO << "--    param_file: " << param_file;

    if (!Configure(param_file)) {
        AERROR << "failed to load ncut config.";
        return false;
    }

    _outliers.reset(new std::vector<ObjectPtr>);
    if (!_outliers) {
        AERROR << "Failed to reset outliers.";
        return false;
    }
#pragma omp parallel
    { num_threads_ = omp_get_num_threads(); }

    AINFO << "number threads " << num_threads_;
    _segmentors.resize(num_threads_);
    for (int i = 0; i < num_threads_; ++i) {
        _segmentors[i].reset(new NCut);
        if (!(_segmentors[i]->Init(ncut_param_))) {
            AERROR << "failed to init NormalizedCut " << i << ".";
            return false;
        }
    }

    roi_cloud_ = base::PointFCloudPool::Instance().Get();
    roi_world_cloud_ = base::PointDCloudPool::Instance().Get();

#ifdef DEBUG_NCUT
    _viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
    _viewer->setBackgroundColor(0, 0, 0);
    _viewer->addCoordinateSystem(1.0);
    _viewer->initCameraParameters();
    _viewer_count = 0;
    _rgb_cloud = CPointCloudPtr(new CPointCloud);
#endif

    AINFO << "NCutSegmentation init success, num_threads: " << num_threads_;
    return true;
}

bool NCutSegmentation::Configure(std::string param_file) {
    NCutSegmentationParam seg_param_;
    // get cnnseg params
    ACHECK(GetProtoFromFile(param_file, &seg_param_)) << "Failed to parse CNNSegParam config file." << param_file;
    grid_radius_ = seg_param_.grid_radius();
    height_threshold_ = seg_param_.height_threshold();
    partition_cell_size_ = seg_param_.partition_cell_size();
    vehicle_filter_cell_size_ = seg_param_.vehicle_filter_cell_size();
    pedestrian_filter_cell_size_ = seg_param_.pedestrian_filter_cell_size();
    outlier_length_ = seg_param_.outlier_length();
    outlier_width_ = seg_param_.outlier_width();
    outlier_height_ = seg_param_.outlier_height();
    outlier_min_num_points_ = seg_param_.outlier_min_num_points();
    remove_ground_ = seg_param_.remove_ground_points();
    remove_roi_ = seg_param_.remove_roi();
    ncut_param_ = seg_param_.ncut_param();
    do_classification_ = seg_param_.do_classification();
    AINFO << "NCut Segmentation " << seg_param_.DebugString();
    return true;
}

bool NCutSegmentation::Detect(const LidarDetectorOptions& options, LidarFrame* frame) {
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

    AINFO << "input point cloud: " << original_cloud_->size() << " points";
#ifdef DEBUG_NCUT
    VisualizePointCloud(original_cloud_);
#endif

    base::PointFCloudPtr cloud_above_ground(new base::PointFCloud);
    if (remove_ground_) {
        AINFO << "remove ground";
        CloudMask mask;
        mask.Set(original_cloud_->size(), 0);
        mask.AddIndicesOfIndices(lidar_frame_ref_->roi_indices, lidar_frame_ref_->non_ground_indices, 1);
        mask.GetValidIndices(&lidar_frame_ref_->secondary_indices);
    }

    // if used as secondary segmentor, got from cloud directly
    cloud_above_ground->CopyPointCloud(*lidar_frame_ref_->cloud, lidar_frame_ref_->secondary_indices);

#ifdef DEBUG_NCUT
    AINFO << "filter ground, elapsed time: " << omp_get_wtime() - start_t << cloud_above_ground->size()
          << " points left";
    start_t = omp_get_wtime();
    VisualizePointCloud(cloud_above_ground);
#endif

    // .3 filter vehicle
    base::PointFCloudPtr cloud_after_car_filter = cloud_above_ground;
    if (filter_vehicle_) {
        ObstacleFilter(cloud_above_ground, vehicle_filter_cell_size_, false, &cloud_after_car_filter, segments);
    }
#ifdef DEBUG_NCUT
    AINFO << "filter vehicle, elapsed time: " << omp_get_wtime() - start_t
          << "filter vehicle: " << cloud_after_car_filter->size() << " points left";
    start_t = omp_get_wtime();
    VisualizePointCloud(cloud_after_car_filter);
#endif

    // .4 filter pedestrian
    base::PointFCloudPtr cloud_after_people_filter = cloud_after_car_filter;
    if (filter_pedestrian_) {
        ObstacleFilter(
                cloud_after_car_filter, pedestrian_filter_cell_size_, true, &cloud_after_people_filter, segments);
    }

#ifdef DEBUG_NCUT
    AINFO << "filter pedestrian, elapsed time: " << omp_get_wtime() - start_t;
    start_t = omp_get_wtime();
    AINFO << "filter pedestrian: " << cloud_after_people_filter->size() << " points left";
    VisualizePointCloud(cloud_after_people_filter);
    AINFO << "after filter car/pedestrian #segments " << segments->size();
// VisualizeSegments(*segments);
#endif

    // .5 partition into small regions
    std::vector<base::PointFCloudPtr> cloud_components;
    PartitionConnectedComponents(cloud_after_people_filter, partition_cell_size_, &cloud_components);

#ifdef DEBUG_NCUT
    AINFO << "partition small regions, elapsed time: " << omp_get_wtime() - start_t;
    start_t = omp_get_wtime();
    AINFO << "partition " << cloud_components.size() << " components";
#endif

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
        base::ObjectPtr obj(new base::Object);
        obj->lidar_supplement.cloud = *pc;
        _outliers->push_back(obj);
    }
#ifdef DEBUG_NCUT
    AINFO << "filter outlier, elapsed time: " << omp_get_wtime() - start_t;
    start_t = omp_get_wtime();
#endif
    // .6 graph cut each
    std::vector<std::vector<base::PointFCloudPtr>> threads_segment_pcs(num_threads_);
    std::vector<std::vector<std::string>> threads_segment_labels(num_threads_);
    std::vector<std::vector<base::PointFCloudPtr>> threads_outlier_pcs(num_threads_);
    // .6.1 process each component in parallel
    omp_set_num_threads(num_threads_);
#pragma omp parallel
    {
        int tid = omp_get_thread_num();
        std::shared_ptr<NCut> my_ncut = _segmentors[tid];
        std::vector<base::PointFCloudPtr>& my_segment_pcs = threads_segment_pcs[tid];
        std::vector<std::string>& my_segment_labels = threads_segment_labels[tid];
        std::vector<base::PointFCloudPtr>& my_outlier_pcs = threads_outlier_pcs[tid];

#pragma omp for schedule(guided)
        for (size_t i = 0; i < cloud_tbd.size(); ++i) {
            my_ncut->Segment(cloud_components[cloud_tbd[i]]);
            ADEBUG << "after segment with num segments" << my_ncut->NumSegments();
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
#ifdef DEBUG_NCUT
    ADEBUG << "parallel normalized cut, elapsed time: " << omp_get_wtime() - start_t;
    start_t = omp_get_wtime();
#endif
    // .6.2 aggregate results
    std::vector<int> segment_offset(num_threads_, static_cast<int>(segments->size()));
    for (int i = 1; i < num_threads_; ++i) {
        segment_offset[i] = segment_offset[i - 1] + static_cast<int>(threads_segment_pcs[i - 1].size());
    }
    int new_num_segments
            = static_cast<int>(threads_segment_pcs[num_threads_ - 1].size()) + segment_offset[num_threads_ - 1];
    segments->resize(new_num_segments);
#pragma omp parallel for
    for (int i = 0; i < num_threads_; ++i) {
        int offset = segment_offset[i];
        for (size_t j = 0; j < threads_segment_pcs[i].size(); ++j) {
            base::ObjectPtr& obj_ptr = (*segments)[offset + j];
            obj_ptr.reset(new base::Object());
            obj_ptr->lidar_supplement.cloud = *threads_segment_pcs[i][j];

            if (do_classification_) {
                obj_ptr->type = Label2Type(threads_segment_labels[i][j]);
            }
        }
    }
    std::vector<int> outlier_offset(num_threads_, static_cast<int>(_outliers->size()));
    for (int i = 1; i < num_threads_; ++i) {
        outlier_offset[i] = outlier_offset[i - 1] + static_cast<int>(threads_outlier_pcs[i - 1].size());
    }
    int new_num_outliers
            = static_cast<int>(threads_outlier_pcs[num_threads_ - 1].size()) + outlier_offset[num_threads_ - 1];
    _outliers->resize(new_num_outliers);
#pragma omp parallel for
    for (int i = 0; i < num_threads_; ++i) {
        int offset = outlier_offset[i];
        for (size_t j = 0; j < threads_outlier_pcs[i].size(); ++j) {
            base::ObjectPtr& obj_ptr = (*_outliers)[offset + j];
            obj_ptr.reset(new base::Object);
            obj_ptr->lidar_supplement.cloud = *threads_outlier_pcs[i][j];
        }
    }
    AINFO << "aggregate results, elapsed time: " << omp_get_wtime() - start_t;
    AINFO << "final #segments " << segments->size();
    AINFO << "final #outliers " << _outliers->size();

#ifdef DEBUG_NCUT
    VisualizeSegments(*segments);
    VisualizeSegments(*_outliers);
#endif

    return true;
}

base::ObjectType NCutSegmentation::Label2Type(const std::string& label) {
    if (label == "unknown") {
        return base::ObjectType::UNKNOWN;
    }
    if (label == "nonMot") {
        return base::ObjectType::BICYCLE;
    }
    if (label == "pedestrian") {
        return base::ObjectType::PEDESTRIAN;
    }
    if (label == "smallMot") {
        return base::ObjectType::VEHICLE;
    }
    return base::ObjectType::UNKNOWN;
}

void NCutSegmentation::PartitionConnectedComponents(
        const base::PointFCloudPtr& in_cloud,
        float cell_size,
        std::vector<base::PointFCloudPtr>* out_clouds) {
    std::vector<base::PointFCloudPtr>& temp_clouds = *out_clouds;
    FloodFill FFfilter(grid_radius_, cell_size);
    std::vector<std::vector<int>> component_points;
    std::vector<int> num_cells_per_components;
    FFfilter.GetSegments(in_cloud, &component_points, &num_cells_per_components);
    temp_clouds.resize(component_points.size());
    for (size_t i = 0; i < component_points.size(); ++i) {
        temp_clouds[i] = base::PointFCloudPtr(new base::PointFCloud(*in_cloud, component_points[i]));
    }
}

void NCutSegmentation::ObstacleFilter(
        const base::PointFCloudPtr& in_cloud,
        float cell_size,
        bool filter_pedestrian_only,
        base::PointFCloudPtr* out_cloud,
        std::vector<base::ObjectPtr>* segments) {
    FloodFill FFfilter(grid_radius_, cell_size);
    std::vector<std::vector<int>> component_points;
    std::vector<int> num_cells_per_components;
    FFfilter.GetSegments(in_cloud, &component_points, &num_cells_per_components);

#ifdef DEBUG_NCUT
    AINFO << "flood fill: " << component_points.size() << " components";
// VisualizeComponents(in_cloud, component_points);
#endif

    const unsigned int min_num_points = 50;
    const int num_components = static_cast<int>(component_points.size());
    std::vector<std::string> component_labels(num_components, "unknown");
    int tid = 0;
    for (int i = 0; i < num_components; ++i) {
        if (component_points[i].size() > min_num_points) {
            base::PointFCloudPtr pc = base::PointFCloudPtr(new base::PointFCloud(*in_cloud, component_points[i]));
            std::string label = _segmentors[tid]->GetPcRoughLabel(pc, filter_pedestrian_only);
            AINFO << "before: component id: " << i << ", label: " << label;
            if (filter_pedestrian_only) {
                label = (label == "pedestrian" || label == "nonMot") ? label : "unknown";
            }
            AINFO << "after: component id: " << i << ", label: " << label;
            component_labels[i] = label;
        }
    }  // end of for

    AINFO << "classification done";
    std::vector<int> remaining_pids;
    std::vector<int> obstacle_components;
    for (int i = 0; i < num_components; ++i) {
        if (component_labels[i] != "unknown") {
            obstacle_components.push_back(i);
        } else {
            remaining_pids.insert(remaining_pids.begin(), component_points[i].begin(), component_points[i].end());
        }
    }
    AINFO << "obstacle_filter: filter unknown out, obstacle_components.size = " << obstacle_components.size()
          << ", remaining_pids.size = " << remaining_pids.size();
    int offset = static_cast<int>(segments->size());
    segments->resize(offset + obstacle_components.size());
    for (size_t i = 0; i < obstacle_components.size(); ++i) {
        int cid = obstacle_components[i];
        ObjectPtr& object_ptr = (*segments)[offset + i];
        object_ptr.reset(new base::Object);
        object_ptr->lidar_supplement.cloud.CopyPointCloud(*in_cloud, component_points[cid]);
    }
    *out_cloud = base::PointFCloudPtr(new base::PointFCloud(*in_cloud, remaining_pids));
}

bool NCutSegmentation::IsOutlier(const base::PointFCloudPtr& in_cloud) {
    size_t min_num_points = std::max(outlier_min_num_points_, 1);
    if (in_cloud->size() < min_num_points) {
        return true;
    }
    float x_max = -std::numeric_limits<float>::max();
    float y_max = -std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    float x_min = std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max();
    float z_min = std::numeric_limits<float>::max();
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

#ifdef DEBUG_NCUT
void NCutSegmentation::VisualizePointCloud(const base::PointFCloudPtr& cloud) {
    // _viewer->removePointCloud(_viewer_id, 0);
    _viewer->removeAllPointClouds(0);
    _viewer->removeAllShapes(0);
    _rgb_cloud->clear();
    for (size_t i = 0; i < cloud->size(); ++i) {
        CPoint pt;
        pt.x = (*cloud)[i].x;
        pt.y = (*cloud)[i].y;
        pt.z = (*cloud)[i].z;
        pt.r = 255;
        pt.g = 255;
        pt.b = 255;
        _rgb_cloud->push_back(pt);
    }
    snprintf(_viewer_id, sizeof(_viewer_id), "vis%06d", _viewer_count++);
    _viewer->addPointCloud(_rgb_cloud, _viewer_id, 0);
    _viewer->spin();
}

void NCutSegmentation::VisualizeSegments(const std::vector<base::ObjectPtr>& segments) {
    // _viewer->removePointCloud(_viewer_id, 0);
    unsigned int seed = 0;
    _viewer->removeAllPointClouds(0);
    _viewer->removeAllShapes(0);
    _rgb_cloud->clear();
    for (size_t i = 0; i < segments.size(); ++i) {
        int red = 50 + rand_r(&seed) % 206;
        int green = 50 + rand_r(&seed) % 206;
        int blue = 50 + rand_r(&seed) % 206;
        const base::PointFCloud& pc = segments[i]->lidar_supplement.cloud;
        for (size_t j = 0; j < pc.size(); ++j) {
            CPoint pt;
            pt.x = pc[j].x;
            pt.y = pc[j].y;
            pt.z = pc[j].z;
            pt.r = static_cast<uint8_t>(red);
            pt.g = static_cast<uint8_t>(green);
            pt.b = static_cast<uint8_t>(blue);
            _rgb_cloud->push_back(pt);
        }
    }
    snprintf(_viewer_id, sizeof(_viewer_id), "vis%06d", _viewer_count++);
    _viewer->addPointCloud(_rgb_cloud, _viewer_id, 0);
    _viewer->spin();
}

void NCutSegmentation::VisualizeComponents(
        const base::PointFCloudPtr& cloud,
        const std::vector<std::vector<int>>& component_points) {
    // _viewer->removePointCloud(_viewer_id, 0);
    unsigned int seed = 0;
    _viewer->removeAllPointClouds(0);
    _viewer->removeAllShapes(0);
    _rgb_cloud->clear();
    std::vector<CPoint> centers(component_points.size());
    for (size_t i = 0; i < component_points.size(); ++i) {
        int red = 50 + rand_r(&seed) % 206;
        int green = 50 + rand_r(&seed) % 206;
        int blue = 50 + rand_r(&seed) % 206;
        const int num_points = static_cast<int>(component_points[i].size());
        CPoint center;
        center.x = 0.f;
        center.y = 0.f;
        center.z = 0.f;
        for (size_t j = 0; j < component_points[i].size(); ++j) {
            CPoint pt;
            int pid = component_points[i][j];
            pt.x = (*cloud)[pid].x;
            pt.y = (*cloud)[pid].y;
            pt.z = (*cloud)[pid].z;
            pt.r = static_cast<uint8_t>(red);
            pt.g = static_cast<uint8_t>(green);
            pt.b = static_cast<uint8_t>(blue);
            _rgb_cloud->push_back(pt);
            center.x += pt.x;
            center.y += pt.y;
            center.z += pt.z;
        }
        center.x /= static_cast<float>(num_points);
        center.y /= static_cast<float>(num_points);
        center.z /= static_cast<float>(num_points);
        centers[i] = center;
    }
    snprintf(_viewer_id, sizeof(_viewer_id), "vis%06d", _viewer_count++);
    _viewer->addPointCloud(_rgb_cloud, _viewer_id, 0);
    for (size_t i = 0; i < component_points.size(); ++i) {
        char text[256];
        char text_id[256];
        snprintf(text, sizeof(text), "%zu", i);
        snprintf(text_id, sizeof(text_id), "c%zu", i);
        _viewer->addText3D(text, centers[i], 0.3, 1.0, 1.0, 1.0, text_id, 0);
    }
    _viewer->spin();
}
#endif

PERCEPTION_REGISTER_LIDARDETECTOR(NCutSegmentation);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
