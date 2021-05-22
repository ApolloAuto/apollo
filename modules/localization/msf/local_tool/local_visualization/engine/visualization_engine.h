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

/**
 * @file
 * @brief The engine for localization visualization.
 */
#pragma once

#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"

#include "modules/common/util/eigen_defs.h"

namespace apollo {
namespace localization {
namespace msf {

/**
 * @struct LocalizatonInfo
 * @brief The data structure to store info of a localization
 */
struct LocalizatonInfo {
  void set(const Eigen::Translation3d &location,
           const Eigen::Quaterniond &attitude, const Eigen::Vector3d &std_var,
           const std::string &description, const double timestamp,
           const unsigned int frame_id) {
    set(location, attitude, description, timestamp, frame_id);
    this->std_var = std_var;
    is_has_std = true;
  }

  void set(const Eigen::Translation3d &location,
           const Eigen::Quaterniond &attitude, const std::string &description,
           const double timestamp, const unsigned int frame_id) {
    set(location, description, timestamp, frame_id);
    this->attitude = attitude;
    this->pose = location * attitude;
    is_has_attitude = true;
  }

  void set(const Eigen::Translation3d &location, const Eigen::Vector3d &std_var,
           const std::string &description, const double timestamp,
           const unsigned int frame_id) {
    set(location, description, timestamp, frame_id);
    this->std_var = std_var;
    is_has_std = true;
  }

  void set(const Eigen::Translation3d &location, const std::string &description,
           const double timestamp, const unsigned int frame_id) {
    this->attitude = Eigen::Quaterniond::Identity();
    this->location = location;
    this->pose = location * attitude;
    this->description = description;
    this->timestamp = timestamp;
    this->frame_id = frame_id;
    is_valid = true;
  }

  Eigen::Translation3d location;
  Eigen::Quaterniond attitude;
  Eigen::Affine3d pose;
  Eigen::Vector3d std_var = {0.01, 0.01, 0.01};
  std::string description;
  double timestamp = 0;
  unsigned int frame_id = 0;
  bool is_valid = false;
  bool is_has_attitude = false;
  bool is_has_std = false;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct VisualMapParam
 * @brief The data structure to store parameters of a map
 */
struct VisualMapParam {
  void set(const std::vector<float> &map_resolutions,
           const unsigned int map_node_size_x,
           const unsigned int map_node_size_y, const double map_min_x,
           const double map_min_y, const double map_max_x,
           const double map_max_y) {
    this->map_resolutions = map_resolutions;
    this->map_node_size_x = map_node_size_x;
    this->map_node_size_y = map_node_size_y;
    this->map_min_x = map_min_x;
    this->map_min_y = map_min_y;
    this->map_max_x = map_max_x;
    this->map_max_y = map_max_y;
  }

  /**@brief The pixel resolutions in the map in meters. */
  std::vector<float> map_resolutions;
  /**@brief The map node size in pixels. */
  unsigned int map_node_size_x = 0;
  /**@brief The map node size in pixels. */
  unsigned int map_node_size_y = 0;
  /**@brief The minimum and maximum UTM range in the map. */
  double map_min_x = 0;
  double map_min_y = 0;
  double map_max_x = 0;
  double map_max_y = 0;
};

/**
 * @struct MapImageKey
 * @brief The key structure of a map image .
 */
struct MapImageKey {
  bool operator<(const MapImageKey &key) const;

  unsigned int level = 0;
  int zone_id = 0;
  unsigned int node_north_id = 0;
  unsigned int node_east_id = 0;
};

/**
 * @class MapImageCache
 * @brief The cache to load map images.
 */
class MapImageCache {
 public:
  typedef std::list<std::pair<MapImageKey, cv::Mat>>::iterator ListIterator;

 public:
  explicit MapImageCache(int capacity) : _capacity(capacity) {}
  bool Get(const MapImageKey &key, cv::Mat *image);
  void Set(const MapImageKey &key, const cv::Mat &image);

 private:
  unsigned int _capacity;
  std::map<MapImageKey, ListIterator> _map;
  std::list<std::pair<MapImageKey, cv::Mat>> _list;
};

/**
 * @class VisualizationEngine
 * @brief The engine to draw all elements for visualization.
 */
class VisualizationEngine {
 public:
  VisualizationEngine();
  ~VisualizationEngine() = default;

 public:
  bool Init(const std::string &map_folder, const std::string &map_visual_folder,
            const VisualMapParam &map_param, const unsigned int resolution_id,
            const int zone_id, const Eigen::Affine3d &extrinsic,
            const unsigned int loc_info_num = 1);
  void Visualize(::apollo::common::EigenVector<LocalizatonInfo> &&loc_infos,
                 const ::apollo::common::EigenVector3dVec &cloud);
  void SetAutoPlay(bool auto_play);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void Preprocess(const std::string &map_folder,
                  const std::string &map_visual_folder);
  void Draw();
  void DrawLoc(const cv::Point &bias);
  void DrawStd(const cv::Point &bias);
  void DrawCloud(const cv::Point &bias);
  void DrawTrajectory(const cv::Point &bias);
  void DrawLegend();
  void DrawInfo();
  void DrawTips();

  void UpdateLevel();
  /**@brief Generate multi resolution images from origin map node images.*/
  void GenerateMutiResolutionImages(const std::vector<std::string> &src_files,
                                    const int base_path_length,
                                    const std::string &dst_folder);
  void InitOtherParams(const int x_min, const int y_min, const int x_max,
                       const int y_max, const int level,
                       const std::string &path);
  bool InitOtherParams(const std::string &params_file);

  /**@brief Project point cloud ti mat.*/
  void CloudToMat(const Eigen::Affine3d &cur_pose,
                  const Eigen::Affine3d &velodyne_extrinsic,
                  const ::apollo::common::EigenVector3dVec &cloud,
                  cv::Mat *cloud_img, cv::Mat *cloud_img_mask);
  void CoordToImageKey(const Eigen::Vector2d &coord, MapImageKey *key);
  /**@brief Compute grid index in current map given global coordinate.*/
  cv::Point CoordToMapGridIndex(const Eigen::Vector2d &coord,
                                const unsigned int resolution_id,
                                const int stride);
  /**@brief Compute grid index in spcific map node.*/
  cv::Point MapGridIndexToNodeGridIndex(const cv::Point &p);

  bool LoadImageToCache(const MapImageKey &key);

  void RotateImg(const cv::Mat &in_img, cv::Mat *out_img, double angle);

  void SetViewCenter(const double center_x, const double center_y);
  void UpdateViewCenter(const double move_x, const double move_y);
  void SetScale(const double scale);
  void UpdateScale(const double factor);
  bool UpdateCarLocId();
  bool UpdateCarLocId(const unsigned int car_loc_id);
  bool UpdateTrajectoryGroups();
  void ProcessKey(int key);

  inline void QuaternionToEuler(const double quaternion[4], double att[3]) {
    double dcm21 =
        2 * (quaternion[2] * quaternion[3] + quaternion[0] * quaternion[1]);
    double dcm20 =
        2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
    double dcm22 =
        quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] -
        quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
    double dcm01 =
        2 * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);
    double dcm11 =
        quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3];

    att[0] = asin(dcm21);           // the angle rotate respect to X
    att[1] = atan2(-dcm20, dcm22);  // the angle rotate respect to Y
    att[2] = atan2(dcm01, dcm11);   // the angle rotate respect to Z

    return;
  }

 private:
  std::string map_folder_;
  std::string map_visual_folder_;
  VisualMapParam map_param_;
  unsigned int zone_id_ = 50;
  unsigned int resolution_id_ = 0;

  std::string image_visual_resolution_path_;
  std::string image_visual_leaf_path_;

  MapImageCache map_image_cache_;
  cv::Point lt_node_index_;
  cv::Point lt_node_grid_index_;

  std::string window_name_ = "Local Visualizer";
  cv::Mat image_window_;
  cv::Mat big_window_;
  cv::Mat subMat_[3][3];
  cv::Mat tips_window_;

  Eigen::Vector2d _view_center;
  double cur_scale_ = 1.0;
  int cur_stride_ = 1;
  int cur_level_ = 0;
  int max_level_ = 0;
  int max_stride_ = 1;

  bool is_init_ = false;
  bool follow_car_ = true;
  bool auto_play_ = false;

  Eigen::Affine3d car_pose_;
  ::apollo::common::EigenVector3dVec cloud_;
  cv::Mat cloud_img_;
  cv::Mat cloud_img_mask_;
  Eigen::Vector2d cloud_img_lt_coord_;
  Eigen::Affine3d velodyne_extrinsic_;

  unsigned int loc_info_num_ = 1;
  unsigned int car_loc_id_ = 0;
  unsigned int expected_car_loc_id_ = 0;
  ::apollo::common::EigenVector<LocalizatonInfo> cur_loc_infos_;
  std::vector<std::map<double, Eigen::Vector2d>> trajectory_groups_;

  bool is_draw_car_ = true;
  bool is_draw_trajectory_ = true;
  bool is_draw_std_ = true;
  std::vector<cv::Mat> car_img_mats_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
