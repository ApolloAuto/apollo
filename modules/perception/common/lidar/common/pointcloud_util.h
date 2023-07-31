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

#include <string>

#include "pcl/filters/passthrough.h"
#include "pcl/filters/random_sample.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/impl/pcl_base.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/pcl_base.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#define PCL_NO_PRECOMPILE

namespace apollo {
namespace perception {
namespace lidar {

struct PCLPointXYZIT {
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointT>
void DownSampleCloudByVoxelGrid(
    const typename pcl::PointCloud<PointT>::Ptr &input_cloud_ptr,
    typename pcl::PointCloud<PointT>::Ptr output_cloud_ptr, float lx = 0.01f,
    float ly = 0.01f, float lz = 0.01f) {
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(input_cloud_ptr);
  voxel_grid.setLeafSize(lx, ly, lz);
  voxel_grid.filter(*output_cloud_ptr);
}

template <typename PointT>
void FilterPointsOutsideRange(
    const typename pcl::PointCloud<PointT>::Ptr &input_cloud_ptr,
    const std::string &filter_axis, const double &limit_min,
    const double &limit_max,
    typename pcl::PointCloud<PointT>::Ptr output_cloud_ptr) {
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(input_cloud_ptr);
  pass.setFilterFieldName(filter_axis);
  pass.setFilterLimits(limit_min, limit_max);
  pass.filter(*output_cloud_ptr);
}

template <typename PointT>
void LoadPointCloud(const std::string &filename,
                    typename pcl::PointCloud<PointT>::Ptr point_cloud) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(filename, *point_cloud) == -1) {
    throw std::runtime_error("Couldn't read file " + filename);
  }
  std::cout << "Loaded " << point_cloud->width * point_cloud->height
            << " data points from " << filename << std::endl;
}

template <typename PointT>
void DownSamplePointCloud(
    const typename pcl::PointCloud<PointT>::Ptr &intput_cloud_ptr,
    typename pcl::PointCloud<PointT>::Ptr out_cloud_ptr, int num_points) {
  if (num_points <= 0) {
    std::cerr << "Invalid number of points to downsample: " << num_points
              << std::endl;
  }

  pcl::RandomSample<PointT> sampler;
  sampler.setInputCloud(intput_cloud_ptr);
  sampler.setSample(num_points);
  sampler.filter(*out_cloud_ptr);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::PCLPointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      std::uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))
