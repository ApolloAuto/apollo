/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_LOCALIZATION_MSF_EXTRACT_GROUND_PLANE_H_
#define MODULES_LOCALIZATION_MSF_EXTRACT_GROUND_PLANE_H_

#include <cmath>
#include <map>
#include <unordered_map>
#include <vector>

#include "pcl/sample_consensus/impl/ransac.hpp"
#include "pcl/sample_consensus/impl/sac_model_plane.hpp"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_plane.h"

#include "modules/localization/msf/common/util/voxel_grid_covariance_hdmap.h"

namespace apollo {
namespace localization {
namespace msf {
class FeatureXYPlane {
 public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef PointCloudT::Ptr PointCloudPtrT;

 public:
  FeatureXYPlane() {
    min_grid_size_ = 0.5;
    max_grid_size_ = 4.00;
    plane_inlier_distance_ = 0.05;
    min_planepoints_number_ = 60;
    plane_type_degree_ = 80.0;
    below_lidar_height_ = 1.0;
    xy_plane_cloud_ = PointCloudPtrT(new PointCloudT);
    non_xy_plane_cloud_ = PointCloudPtrT(new PointCloudT);
  }

  void SetMinGridSize(double d) { min_grid_size_ = d; }

  void SetMaxGridSize(double d) { max_grid_size_ = d; }

  void SetPlaneInlierDistance(double d) { plane_inlier_distance_ = d; }

  void SetMinPlanepointsNumber(double d) { min_planepoints_number_ = d; }

  void SetPlaneTypeDegree(double d) { plane_type_degree_ = d; }

  void SetBelowLidarHeight(double d) { below_lidar_height_ = d; }

  float CalculateDegree(const Eigen::Vector3f& tmp0,
                        const Eigen::Vector3f& tmp1) {
    float cos_theta = tmp0.dot(tmp1) / (tmp0.norm() * tmp1.norm());
    return std::acos(cos_theta) * 180.0 / M_PI;
  }

  PointCloudPtrT& GetXYPlaneCloud() { return xy_plane_cloud_; }

  PointCloudPtrT& GetNonXYPlaneCloud() { return non_xy_plane_cloud_; }

  void ExtractXYPlane(const PointCloudPtrT& cloud) {
    xy_plane_cloud_.reset(new PointCloudT);
    PointCloudPtrT pointcloud_ptr(new PointCloudT);
    pcl::copyPointCloud<PointT>(*cloud, *pointcloud_ptr);
    int iter_num = log2(max_grid_size_ / min_grid_size_);
    if (iter_num == 0) {
      iter_num = 1;
    }
    std::clock_t plane_time;
    plane_time = std::clock();
    int total_plane_num = 0;
    double power2 = 0.5;  // 2^-1
    for (int iter = 0; iter <= iter_num; ++iter) {
      power2 *= 2;
      double grid_size = max_grid_size_ / power2;
      VoxelGridCovariance<PointT> vgc;
      vgc.setInputCloud(pointcloud_ptr);
      vgc.SetMinPointPerVoxel(min_planepoints_number_);
      vgc.setLeafSize(grid_size, grid_size, grid_size);
      vgc.Filter(false);

      PointCloudT cloud_tmp;
      int plane_num = 0;
      typename std::map<size_t, VoxelGridCovariance<PointT>::Leaf>::iterator it;
      for (it = vgc.GetLeaves().begin(); it != vgc.GetLeaves().end(); it++) {
        if (it->second.GetPointCount() < min_planepoints_number_) {
          cloud_tmp += it->second.cloud_;
          continue;
        }
        PointCloudT cloud_outlier;
        if (GetPlaneFeaturePoint(it->second.cloud_, &cloud_outlier)) {
          cloud_tmp += cloud_outlier;
          plane_num++;
        } else {
          cloud_tmp += it->second.cloud_;
        }
      }
      std::cerr << "the " << iter << " interation: plane_num = " << plane_num
                << std::endl;
      total_plane_num += plane_num;
      pointcloud_ptr.reset(new PointCloudT);
      *pointcloud_ptr = cloud_tmp;
    }

    *non_xy_plane_cloud_ = *pointcloud_ptr;
    plane_time = std::clock() - plane_time;
    std::cerr << "plane_patch takes:"
              << static_cast<double>(plane_time) / CLOCKS_PER_SEC << "sec."
              << std::endl;
    std::cerr << "total_plane_num = " << total_plane_num << std::endl;
    std::cerr << "total_points_num = " << xy_plane_cloud_->points.size()
              << std::endl;
    return;
  }

 private:
  bool GetPlaneFeaturePoint(const PointCloudT& cloud,
                            PointCloudT* cloud_outlier) {
    // ransac plane
    std::vector<int> inliers;
    PointCloudPtrT cloud_new(new PointCloudT);
    *cloud_new = cloud;
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_plane(
        new pcl::SampleConsensusModelPlane<PointT>(cloud_new));
    pcl::RandomSampleConsensus<PointT> ransac(model_plane);
    ransac.setDistanceThreshold(plane_inlier_distance_);
    ransac.computeModel();
    ransac.getInliers(inliers);
    if (static_cast<int>(inliers.size()) < min_planepoints_number_) {
      return false;
    }
    PointCloudPtrT cloud_inlier(new PointCloudT);
    pcl::copyPointCloud<PointT>(*cloud_new, inliers, *cloud_inlier);
    std::vector<int> outliers;
    unsigned int inlier_idx = 0;
    for (unsigned int i = 0; i < cloud_new->points.size(); ++i) {
      if (static_cast<int>(i) < inliers[inlier_idx]) {
        outliers.push_back(i);
      } else {
        inlier_idx++;
      }
    }
    pcl::copyPointCloud<PointT>(*cloud_new, outliers, *cloud_outlier);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_inlier, centroid);

    if (centroid(2) > -below_lidar_height_) {
      return true;
    }

    // get plane's normal (which is normalized)
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    // determin the plane type
    double tan_theta = 0;
    double tan_refer_theta = std::tan(plane_type_degree_ / 180.0 * M_PI);
    if ((std::abs(coeff(2)) > std::abs(coeff(0))) &&
        (std::abs(coeff(2)) > std::abs(coeff(1)))) {
      tan_theta = std::abs(coeff(2)) /
                  std::sqrt(coeff(0) * coeff(0) + coeff(1) * coeff(1));
      if (tan_theta > tan_refer_theta) {
        *xy_plane_cloud_ += *cloud_inlier;
      } else {
        // cloud_outlier += *cloud_inlier;
      }
    }
    return true;
  }

 private:
  // parameters
  double min_grid_size_;
  double max_grid_size_;
  double plane_inlier_distance_;
  int min_planepoints_number_;
  double plane_type_degree_;
  double below_lidar_height_;

  PointCloudPtrT xy_plane_cloud_;
  PointCloudPtrT non_xy_plane_cloud_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_EXTRACT_GROUND_PLANE_H_
