#pragma once

#include "include/lidar_locator.h"
#include "modules/common/log.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_config_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_node_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_pool_2d.h"
#include "localization_params.h"
#include "include/lidar_locator.h"

namespace apollo {
namespace localization {
namespace msf {

struct MapNodeData {
  MapNodeData(int w, int h) {
    width = w;
    height = h;
    intensities = new float[width * height];
    intensities_var = new float[width * height];
    altitudes = new float[width * height];
    count = new unsigned int[width * height];
  }
  ~MapNodeData() {
    delete[] intensities;
    intensities = NULL;
    delete[] intensities_var;
    intensities_var = NULL;
    delete[] altitudes;
    altitudes = NULL;
    delete[] count;
    count = NULL;
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
            unsigned int search_range_x,
            unsigned int search_range_y,
            int zone_id, unsigned int resolution_id = 0);

  void SetVelodyneExtrinsic(const Eigen::Affine3d& pose);
  void SetVehicleHeight(double height);

  void SetValidThreshold(float valid_threashold);

  void SetImageAlignMode(int mode);

  void SetLocalizationMode(int mode);
  
  void SetDeltaYawLimit(double limit);

  void SetDeltaPitchRollLimit(double limit);

  int Update(unsigned int frame_idx, const Eigen::Affine3d& pose,
             const Eigen::Vector3d velocity, const LidarFrame& lidar_frame);

  void GetResult(Eigen::Affine3d *location, Eigen::Matrix3d *covariance);

  void GetLocalizationDistribution(Eigen::MatrixXd *distribution);

 protected:
  void ComposeMapNode(const Eigen::Vector3d& trans);
 
  void RefineAltitudeFromMap(Eigen::Affine3d *pose);

 protected:
  LidarLocator _lidar_locator;
  int _search_range_x;
  int _search_range_y;
  int _node_size_x;
  int _node_size_y;
  double _resolution;
  MapNodeData* _lidar_map_node;

  LossyMapConfig _config;
  LossyMap _map;
  LossyMapNodePool _map_preload_node_pool;
  Eigen::Vector2d _map_left_top_corner;
  unsigned int _resolution_id;
  int _zone_id;
  bool _is_map_loaded;

  double _vehicle_lidar_height;
  double _pre_vehicle_ground_height;
  bool _is_pre_ground_height_valid;
  Eigen::Affine3d _velodyne_extrinsic;
};

} // msf
} // localization
} // apollo