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

#include "cyber/common/log.h"
#include "include/lidar_locator.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_node.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_pool.h"

namespace apollo {
namespace localization {
namespace msf {

struct LidarFrame {
  LidarFrame() : measurement_time(0.0) {}
  double measurement_time;  // unix time
  std::vector<double> pt_xs;
  std::vector<double> pt_ys;
  std::vector<double> pt_zs;
  std::vector<unsigned char> intensities;
};

struct MapNodeData {
  MapNodeData(const int w, const int h)
      : width(w),
        height(h),
        intensities(new float[width * height]),
        intensities_var(new float[width * height]),
        altitudes(new float[width * height]),
        count(new unsigned int[width * height]) {}
  ~MapNodeData() {
    delete[] intensities;
    intensities = nullptr;
    delete[] intensities_var;
    intensities_var = nullptr;
    delete[] altitudes;
    altitudes = nullptr;
    delete[] count;
    count = nullptr;
  }
  int width;
  int height;
  float* intensities;
  float* intensities_var;
  float* altitudes;
  unsigned int* count;
};

class LocalizationLidar {
 public:
  typedef apollo::localization::msf::pyramid_map::PyramidMap PyramidMap;
  typedef apollo::localization::msf::pyramid_map::MapNodeIndex MapNodeIndex;
  typedef apollo::localization::msf::pyramid_map::PyramidMapNode PyramidMapNode;
  typedef apollo::localization::msf::pyramid_map::PyramidMapNodePool
      PyramidMapNodePool;
  typedef apollo::localization::msf::pyramid_map::PyramidMapMatrix
      PyramidMapMatrix;
  typedef apollo::localization::msf::pyramid_map::PyramidMapConfig
      PyramidMapConfig;
  typedef apollo::localization::msf::pyramid_map::FloatMatrix FloatMatrix;
  typedef apollo::localization::msf::pyramid_map::UIntMatrix UIntMatrix;

 public:
  /**@brief The constructor. */
  LocalizationLidar();
  /**@brief The deconstructor. */
  ~LocalizationLidar();

  bool Init(const std::string& map_path, const unsigned int search_range_x,
            const unsigned int search_range_y, const int zone_id,
            const unsigned int resolution_id = 0);

  void SetVelodyneExtrinsic(const Eigen::Affine3d& pose);

  void SetVehicleHeight(double height);

  void SetValidThreshold(float valid_threashold);

  void SetImageAlignMode(int mode);

  void SetLocalizationMode(int mode);

  void SetDeltaYawLimit(double limit);

  void SetDeltaPitchRollLimit(double limit);

  int Update(const unsigned int frame_idx, const Eigen::Affine3d& pose,
             const Eigen::Vector3d velocity, const LidarFrame& lidar_frame,
             bool use_avx = false);

  void GetResult(Eigen::Affine3d* location, Eigen::Matrix3d* covariance,
                 double* location_score);

  void GetLocalizationDistribution(Eigen::MatrixXd* distribution);

 protected:
  void ComposeMapNode(const Eigen::Vector3d& trans);

  void RefineAltitudeFromMap(Eigen::Affine3d* pose);

 protected:
  LidarLocator* lidar_locator_;
  int search_range_x_ = 0;
  int search_range_y_ = 0;
  int node_size_x_ = 0;
  int node_size_y_ = 0;
  double resolution_ = 0.125;
  MapNodeData* lidar_map_node_;

  PyramidMapConfig config_;
  PyramidMap map_;
  PyramidMapNodePool map_node_pool_;
  Eigen::Vector2d map_left_top_corner_;
  unsigned int resolution_id_ = 0;
  int zone_id_ = 50;
  bool is_map_loaded_ = false;

  double vehicle_lidar_height_ = 1.7;
  double pre_vehicle_ground_height_ = 0.0;
  bool is_pre_ground_height_valid_ = false;
  Eigen::Affine3d velodyne_extrinsic_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
