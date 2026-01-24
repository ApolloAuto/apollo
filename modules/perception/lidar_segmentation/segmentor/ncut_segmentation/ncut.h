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

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/opencv.hpp>

#include "Eigen/Core"

#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/proto/ncut_config.pb.h"
#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/proto/ncut_param.pb.h"

#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/flood_fill.h"
#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/lr_classifier.h"

namespace apollo {
namespace perception {
namespace lidar {

class NCut {
public:
    NCut();
    ~NCut();
    /**
     * @brief Init Ncut according to parameters
     *
     * @param param
     * @return true
     * @return false
     */
    bool Init(const NCutParam& param);
    /**
     * @brief Get segmentation number
     *
     * @return int
     */
    int NumSegments() const {
        return static_cast<int>(_segment_pids.size());
    }
    /**
     * @brief Get the segment label
     *
     * @param sid segment index
     * @return std::string
     */
    std::string GetSegmentLabel(int sid) const {
        return _segment_labels[sid];
    }
    /**
     * @brief Get the segment size
     *
     * @param sid segment index
     * @param length box length
     * @param width box width
     * @param height box height
     */
    void GetSegmentSize(int sid, float* length, float* width, float* height) const {
        NcutBoundingBox box = _segment_bbox[sid];
        *length = std::get<1>(box) - std::get<0>(box);
        *width = std::get<3>(box) - std::get<2>(box);
        *height = std::get<5>(box) - std::get<4>(box);
    }
    /**
     * @brief Get the segment point cloud
     *
     * @param sid segment index
     * @return base::PointFCloudPtr
     */
    base::PointFCloudPtr GetSegmentPointCloud(int sid) const {
        base::PointFCloudPtr pc = base::PointFCloudPtr(new base::PointFCloud(*_cloud_obstacles, _segment_pids[sid]));
        return pc;
    }
    /**
     * @brief Segmen main entrance
     *
     * @param cloud
     */
    void Segment(base::PointFCloudConstPtr cloud);
    /**
     * @brief Get the label of point cloud
     *
     * @param cloud cluster of point cloud
     * @param only_check_pedestrian
     * @return std::string
     */
    std::string GetPcRoughLabel(const base::PointFCloudPtr& cloud, bool only_check_pedestrian);
    /**
     * @brief Get the segment size
     *
     * @param cloud segment of pointcloud
     * @param length object length
     * @param width object width
     * @param height object height
     */
    void GetSegmentRoughSize(const base::PointFCloudPtr& cloud, float* length, float* width, float* height);

private:
    struct gridIndex {
        int irow;
        int jcol;
    };

    // x_min, x_max, y_min, y_max, z_min, z_max;
    typedef std::tuple<float, float, float, float, float, float> NcutBoundingBox;
    base::PointFCloudPtr _cloud_obstacles;
    // super pixels related
    float _grid_radius;
    float _super_pixel_cell_size;
    std::unique_ptr<LRClassifier> _classifier;
    // felzenszwalb
    double _felzenszwalb_sigma;
    double _felzenszwalb_k;
    int _felzenszwalb_min_size;
    // graph cut related
    double _sigma_feature;
    double _sigma_space;
    double _connect_radius;
    int _num_cuts;
    float _ncuts_stop_threshold;
    double _ncuts_enable_classifier_threshold;
    // component (cluster) information
    std::vector<std::vector<int>> _cluster_points;
    // x_min, x_max, y_min, y_max, z_min, z_max;
    std::vector<NcutBoundingBox> _cluster_bounding_box;
    std::vector<std::string> _cluster_labels;
    // skeleton related
    float _skeleton_cell_size;  // skeleton sample size
    int _patch_size;
    cv::Mat _cv_feature_map;
    FloodFill _ff_feature_grid;
    std::vector<Eigen::MatrixXf> _cluster_skeleton_points;
    std::vector<Eigen::MatrixXf> _cluster_skeleton_features;
    // merge overlap
    double _overlap_factor;
    // final segments, each vector contains
    std::vector<std::vector<int>> _segment_pids;
    std::vector<std::string> _segment_labels;
    std::vector<NcutBoundingBox> _segment_bbox;
    std::vector<std::vector<int>> _outlier_pids;

    void SampleByGrid(
            const std::vector<int>& point_gids,
            Eigen::MatrixXf* skeleton_coords,
            Eigen::MatrixXf* skeleton_feature);

    void PrecomputeAllSkeletonAndBbox();

    bool Configure(const NCutParam& ncut_param_);

    void SuperPixelsFloodFill(
            base::PointFCloudConstPtr cloud,
            float radius,
            float cell_size,
            std::vector<std::vector<int>>* super_pixels);

    // super pixels
    void BuildAverageHeightMap(
            base::PointFCloudConstPtr cloud,
            const FloodFill& ff_map,
            cv::Mat* cv_height_map,
            std::vector<gridIndex>* point_pixel_indices);

    // skeleton
    void GetPatchFeature(const Eigen::MatrixXf& points, Eigen::MatrixXf* features);

    // bounding box
    NcutBoundingBox ComputeClusterBoundingBox(const std::vector<int>& point_gids);

    std::string GetPcLabel(const base::PointFCloudPtr& cloud);

    void NormalizedCut(
            float ncuts_threshold,
            bool use_classifier,
            std::vector<std::vector<int>>* segment_clusters,
            std::vector<std::string>* segment_labels);

    void ComputeSkeletonWeights(Eigen::MatrixXf* weights);

    float GetMinNcuts(
            const Eigen::MatrixXf& in_weights,
            const std::vector<int>* in_clusters,
            std::vector<int>* seg1,
            std::vector<int>* seg2);

    void LaplacianDecomposition(const Eigen::MatrixXf& weights, Eigen::MatrixXf* eigenvectors);

    bool ComputeSquaredSkeletonDistance(
            const Eigen::MatrixXf& in1_points,
            const Eigen::MatrixXf& in1_features,
            const Eigen::MatrixXf& in2_points,
            const Eigen::MatrixXf& in2_features,
            float* dist_point,
            float* dist_feature);

    bool IsMovableObstacle(const std::vector<int>& cluster_ids, std::string* label);

    inline bool IsPotentialPedestrianSize(float length, float width) {
        return ((length > 0.5 && length < 1.5) && (width > 0.3 && width < 1));
    }

    inline bool IsPotentialBicyclistSize(float length, float width) {
        return ((length > 1 && length < 2.5) && (width > 0.3 && width < 1.5));
    }

    inline bool IsPotentialCarSize(float length, float width) {
        // return ( (length > 1.0 && length < 20.0) && (width > 1.0 && width < 3.5)
        // );
        return ((length > 1.0 && length < 8.0) && (width > 1.0 && width < 3.5));
    }

    int GetComponentBoundingBox(const std::vector<int>& cluster_ids, NcutBoundingBox* box);

    inline float GetBboxLength(const NcutBoundingBox& box) {
        return (std::get<1>(box) - std::get<0>(box));
    }

    inline float GetBboxWidth(const NcutBoundingBox& box) {
        return (std::get<3>(box) - std::get<2>(box));
    }

    inline float GetBboxHeight(const NcutBoundingBox& box) {
        return (std::get<5>(box) - std::get<4>(box));
    }

    std::string GetClustersLabel(const std::vector<int>& cluster_ids);

    void GetClustersPids(const std::vector<int>& cids, std::vector<int>* pids);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
