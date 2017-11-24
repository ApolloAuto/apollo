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
 * @file visualization_engine.h
 * @brief The engine for localization visualization.
 */
#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_VISUALIZATION_ENGINE_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_VISUALIZATION_ENGINE_H

#include <Eigen/Geometry>
#include <list>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
namespace apollo {
namespace localization {
namespace msf {

/**
 * @struct LocalizatonInfo
 * @brief The data structure to store info of a localization
 */
struct LocalizatonInfo {
  LocalizatonInfo() {
    is_valid = false;
    is_has_attitude = false;
    is_has_std = false;
    timestamp = 0.0;
    frame_id = 0;
    std_var[0] = 0.01;
    std_var[1] = 0.01;
    std_var[1] = 0.01;
  }

  ~LocalizatonInfo() {}

  void set(const Eigen::Translation3d &location,
           const Eigen::Quaterniond &attitude, const Eigen::Vector3d &std_var,
           const std::string &description, const double &timestamp,
           const unsigned int &frame_id) {
    set(location, attitude, description, timestamp, frame_id);
    this->std_var = std_var;
    is_has_std = true;
  }

  void set(const Eigen::Translation3d &location,
           const Eigen::Quaterniond &attitude, const std::string &description,
           const double &timestamp, const unsigned int &frame_id) {
    set(location, description, timestamp, frame_id);
    this->attitude = attitude;
    this->pose = location * attitude;
    is_has_attitude = true;
  }

  void set(const Eigen::Translation3d &location, const Eigen::Vector3d &std_var,
           const std::string &description, const double &timestamp,
           const unsigned int &frame_id) {
    set(location, description, timestamp, frame_id);
    this->std_var = std_var;
    is_has_std = true;
  }

  void set(const Eigen::Translation3d &location, const std::string &description,
           const double &timestamp, const unsigned int &frame_id) {
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
  Eigen::Vector3d std_var;
  std::string description;
  double timestamp;
  unsigned int frame_id;
  bool is_valid;
  bool is_has_attitude;
  bool is_has_std;
};

/**
 * @struct MapImageKey
 * @brief The key structure of a map image .
 */
struct MapImageKey {
  MapImageKey() : level(0), zone_id(0), node_north_id(0), node_east_id(0) {}
  bool operator<(const MapImageKey &key) const;

  unsigned int level;
  int zone_id;
  unsigned int node_north_id;
  unsigned int node_east_id;
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
  bool Get(const MapImageKey &key, cv::Mat &image);
  void Set(const MapImageKey &key, cv::Mat &image);

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
  ~VisualizationEngine();

 public:
  bool Init(const std::string &map_folder, const BaseMapConfig &map_config,
            const unsigned int resolution_id, const int zone_id,
            const Eigen::Affine3d &extrinsic,
            const unsigned int loc_info_num = 1);
  void Visualize(const std::vector<LocalizatonInfo> &loc_infos,
                 const std::vector<Eigen::Vector3d> &cloud);

 private:
  void Preprocess(const std::string &map_folder);
  void Draw();
  void DrawLoc(const cv::Point &bias);
  void DrawStd(const cv::Point &bias);
  void DrawCloud(const cv::Point &bias);
  void DrawTrajectory(const cv::Point &bias);
  void DrawLegend();
  void DrawInfo();

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
                  const std::vector<Eigen::Vector3d> &cloud, cv::Mat &cloud_img,
                  cv::Mat &cloud_img_mask);
  void CoordToImageKey(const Eigen::Vector2d &coord, MapImageKey &key);
  /**@brief Compute grid index in current map given global coordinate.*/
  cv::Point CoordToMapGridIndex(const Eigen::Vector2d &coord,
                                const unsigned int resolution_id,
                                const int stride);
  /**@brief Compute grid index in spcific map node.*/
  cv::Point MapGridIndexToNodeGridIndex(const cv::Point &p);

  bool LoadImageToCache(const MapImageKey &key);

  void RotateImg(cv::Mat &in_img, cv::Mat &out_img, double angle);

  void SetViewCenter(const double center_x, const double center_y);
  void UpdateViewCenter(const double move_x, const double move_y);
  void SetScale(const double scale);
  void UpdateScale(const double factor);
  bool UpdateCarLocId();
  bool UpdateTrajectoryGroups();
  void ProcessKey(int key);

  inline void QuaternionToEuler(const double quaternion[4], double att[3]) {
    double roll_main = 0.0;
    double yaw_main = 0.0;
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
  BaseMapConfig map_config_;
  unsigned int zone_id_;
  unsigned int resolution_id_;

  std::string image_visual_resolution_path_;
  std::string image_visual_leaf_path_;

  MapImageCache map_image_cache_;
  cv::Point lt_node_index_;
  cv::Point lt_node_grid_index_;

  std::string window_name_;
  cv::Mat image_window_;
  cv::Mat big_window_;
  cv::Mat subMat_[3][3];

  Eigen::Vector2d _view_center;
  double cur_scale_;
  int cur_stride_;
  int cur_level_;
  int max_level_;
  int max_stride_;

  bool is_init_;
  bool follow_car_;
  bool auto_play_;

  Eigen::Affine3d car_pose_;
  std::vector<Eigen::Vector3d> cloud_;
  cv::Mat cloud_img_;
  cv::Mat cloud_img_mask_;
  Eigen::Vector2d cloud_img_lt_coord_;
  Eigen::Affine3d velodyne_extrinsic_;

  unsigned int loc_info_num_;
  unsigned int car_loc_id_;
  std::vector<LocalizatonInfo> cur_loc_infos_;
  std::vector<std::map<double, Eigen::Vector2d>> trajectory_groups_;

  bool is_draw_car_;
  std::vector<cv::Mat> car_img_mats_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_VISUALIZATION_ENGINE_H
