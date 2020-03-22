/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/localization/ndt/ndt_locator/ndt_solver.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_node.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_pool.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_matrix.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node_index.h"

#define USE_PRELOAD_MAP_NODE

#ifdef VIS_USE_OPENCV
#define VIS_USE_OPENCV_ON
#endif
#ifdef VIS_USE_OPENCV_ON
#include <opencv2/opencv.hpp>
void color_mapping(float value, float midvalue, unsigned char* r,
                   unsigned char* g, unsigned char* b) {
  if (value > 1.f) {
    value = 1.f;
  } else if (value < 0.f) {
    value = 0.f;
  }
  if (value > midvalue) {
    value = (value - midvalue) / (1.f - midvalue);
    *r = value * 255.0;
    *g = (1.0 - value) * 255.0;
    *b = 0.0;
  } else {
    value /= midvalue;
    *r = 0.0;
    *g = value * 255.0;
    *b = (1 - value) * 255.0;
  }
}
#endif

namespace apollo {
namespace localization {
namespace ndt {

typedef apollo::localization::msf::pyramid_map::NdtMap NdtMap;
typedef apollo::localization::msf::pyramid_map::NdtMapConfig NdtMapConfig;
typedef apollo::localization::msf::pyramid_map::NdtMapNode NdtMapNode;
typedef apollo::localization::msf::pyramid_map::NdtMapCells NdtMapCells;
typedef apollo::localization::msf::pyramid_map::NdtMapNodePool NdtMapNodePool;
typedef apollo::localization::msf::pyramid_map::NdtMapMatrix NdtMapMatrix;
typedef apollo::localization::msf::pyramid_map::MapNodeIndex MapNodeIndex;

struct LidarFrame {
  LidarFrame() : measurement_time(0.0) {}
  double measurement_time;  // unix time
  std::vector<float> pt_xs;
  std::vector<float> pt_ys;
  std::vector<float> pt_zs;
  std::vector<unsigned char> intensities;
};

class LidarLocatorNdt {
 public:
  /**@brief The constructor. */
  LidarLocatorNdt();
  /**@brief The destructor. */
  ~LidarLocatorNdt();

  /**@brief Load map data. */
  void LoadMap(const Eigen::Affine3d& init_location, unsigned int resolution_id,
               int zone_id);
  /**@brief Initialize the locator. */
  void Init(const Eigen::Affine3d& init_location, unsigned int resolution_id,
            int zone_id);
  /**@brief Set the map folder. */
  void SetMapFolderPath(const std::string folder_path);
  /**@brief Set the extrinsic calibration. */
  void SetVelodyneExtrinsic(const Eigen::Affine3d& extrinsic);
  /**@brief Set the lidar height. */
  void SetLidarHeight(double height);

  /**@brief Compose candidate map area. */
  void ComposeMapCells(const Eigen::Vector2d& left_top_coord2d, int zone_id,
                       unsigned int resolution_id, float map_pixel_resolution,
                       const Eigen::Affine3d& inverse_transform);

  /**@brief Set online cloud resolution. */
  void SetOnlineCloudResolution(const float& online_resolution);

  /**@brief Update the histogram filter.
   * param <pose> The localization from the GPS.
   * param <pt3ds> The local 3D points from Velodyne. */
  int Update(unsigned int frame_idx, const Eigen::Affine3d& pose,
             const LidarFrame& lidar_frame);
  /**@brief Get the current optimal pose result. */
  Eigen::Affine3d GetPose() const;
  /*@brief Get the predict location from the odometry motion model*/
  Eigen::Vector3d GetPredictLocation() const;
  /**@brief Get the covariance of estimated location. */
  Eigen::Matrix3d GetLocationCovariance() const;
  /**@brief Is the locator initialized. */
  inline bool IsInitialized() const { return is_initialized_; }
  /**@brief Is the map data loaded. */
  inline bool IsMaploaded() const { return is_map_loaded_; }
  /**@brief Get the locator map. */
  inline const NdtMap& GetMap() const { return map_; }
  /**@brief Get ndt matching score */
  inline double GetFitnessScore() const { return fitness_score_; }

 private:
  /**@brief Whether initialized. */
  bool is_initialized_ = false;
  /**@brief Whether map is loaded. */
  bool is_map_loaded_ = false;

  /**@brief Estimated location. */
  Eigen::Affine3d location_;
  /**@brief The covariance matrix of localtion position. */
  Eigen::Matrix3d location_covariance_;
  /**@brief Predicted location. */
  Eigen::Affine3d predict_location_;
  /**@brief Last input location. */
  Eigen::Affine3d pre_input_location_;
  /**@brief Last estimated location. */
  Eigen::Affine3d pre_estimate_location_;

  /**@brief Resolution id of map. */
  unsigned int resolution_id_ = 0;
  /**@brief Zone id. */
  int zone_id_ = 10;

  /**@brief The height of laser (the distance between laser and ground). */
  double lidar_height_ = 1.7;
  /**@brief Imu height of last frame (z-value). */
  double pre_imu_height_ = 0;
  /**@brief Extrinsic parameter between velodyne and imu. */
  Eigen::Affine3d velodyne_extrinsic_;

  /**@brief Filter size of map. */
  int filter_x_ = 0;
  int filter_y_ = 0;
  /**@brief Online pointclouds resoltion. */
  float proj_reslution_ = 1.0;

  /**@brief The config file of map. */
  NdtMapConfig config_;
  /**@brief ndt map. */
  NdtMap map_;
  /**@brief ndt mapnode pool. */
  NdtMapNodePool map_preload_node_pool_;
  /**@brief Map point cloud. */
  std::vector<Leaf> cell_map_;
  /**brief Map Left top corner.*/
  Eigen::Vector3d map_left_top_corner_;
  /**@brief NDT transform class. */
  NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> reg_;

  /**@brief ndt matching score */
  double fitness_score_ = 0.0;

  /**@brief maximum iterations for ndt matching*/
  int ndt_max_iterations_ = 10;
  double ndt_target_resolution_ = 1.0;
  double ndt_line_search_step_size_ = 0.1;
  double ndt_transformation_epsilon_ = 0.01;
};

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
