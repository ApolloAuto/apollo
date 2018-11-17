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

#include "modules/common/log.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_config_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_node_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_pool_2d.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "include/lidar_locator.h"

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
      : width(w), height(h),
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
typedef apollo::localization::msf::LossyMap2D LossyMap;
typedef apollo::localization::msf::MapNodeIndex MapNodeIndex;
typedef apollo::localization::msf::LossyMapNode2D LossyMapNode;
typedef apollo::localization::msf::LossyMapNodePool2D LossyMapNodePool;
typedef apollo::localization::msf::LossyMapMatrix2D LossyMapMatrix;
typedef apollo::localization::msf::LossyMapCell2D LossyMapCell;
typedef apollo::localization::msf::LossyMapConfig2D LossyMapConfig;

 public:
  /**@brief The constructor. */
  LocalizationLidar();
  /**@brief The deconstructor. */
  ~LocalizationLidar();

  bool Init(const std::string& map_path,
            const unsigned int search_range_x,
            const unsigned int search_range_y,
            const int zone_id, const unsigned int resolution_id = 0);

  void SetVelodyneExtrinsic(const Eigen::Affine3d& pose);
  void SetVehicleHeight(double height);

  void SetValidThreshold(float valid_threashold);

  void SetImageAlignMode(int mode);

  void SetLocalizationMode(int mode);

  void SetDeltaYawLimit(double limit);

  void SetDeltaPitchRollLimit(double limit);

  int Update(const unsigned int frame_idx, const Eigen::Affine3d& pose,
             const Eigen::Vector3d velocity, const LidarFrame& lidar_frame);

  void GetResult(Eigen::Affine3d *location, Eigen::Matrix3d *covariance);

  void GetLocalizationDistribution(Eigen::MatrixXd *distribution);

 protected:
  void ComposeMapNode(const Eigen::Vector3d& trans);

  void RefineAltitudeFromMap(Eigen::Affine3d *pose);

 protected:
  LidarLocator *lidar_locator_;
  int search_range_x_;
  int search_range_y_;
  int node_size_x_;
  int node_size_y_;
  double resolution_;
  MapNodeData* lidar_map_node_;

  LossyMapConfig config_;
  LossyMap map_;
  LossyMapNodePool map_node_pool_;
  Eigen::Vector2d map_left_top_corner_;
  unsigned int resolution_id_;
  int zone_id_;
  bool is_map_loaded_;

  double vehicle_lidar_height_;
  double pre_vehicle_ground_height_;
  bool is_pre_ground_height_valid_;
  Eigen::Affine3d velodyne_extrinsic_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
