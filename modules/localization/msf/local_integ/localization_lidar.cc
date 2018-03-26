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

#include "modules/localization/msf/local_integ/localization_lidar.h"
#include <string>

namespace apollo {
namespace localization {
namespace msf {

LocalizationLidar::LocalizationLidar()
    : _search_range_x(21), _search_range_y(21),
      _node_size_x(1024), _node_size_y(1024),
      _resolution(0.125), _lidar_map_node(NULL),
      _config("lossy_map"), _map(&_config),
      _map_preload_node_pool(25, 8),
      _resolution_id(0),
      _is_map_loaded(false),
      _vehicle_lidar_height(1.7),
      _pre_vehicle_ground_height(0.0),
      _is_pre_ground_height_valid(false),
      _velodyne_extrinsic(Eigen::Affine3d::Identity()) {
  _map_left_top_corner = Eigen::Vector2d::Zero();
}

LocalizationLidar::~LocalizationLidar() {
  if (_lidar_map_node) {
    delete _lidar_map_node;
    _lidar_map_node = NULL;
  }
}

bool LocalizationLidar::Init(const std::string& map_path,
                             unsigned int search_range_x,
                             unsigned int search_range_y,
                             int zone_id, unsigned int resolution_id) {
  // init map
  _resolution_id = resolution_id;
  _zone_id = zone_id;
  if (!_map.SetMapFolderPath(map_path)) {
    LOG(FATAL) << "Reflectance map folder is invalid!";
    return false;
  }
  _map_preload_node_pool.Initial(&(_map.GetConfig()));
  _map.InitThreadPool(1, 6);
  _map.InitMapNodeCaches(12, 24);
  _map.AttachMapNodePool(&_map_preload_node_pool);

  // init locator
  _node_size_x = _map.GetConfig().map_node_size_x_;
  _node_size_y = _map.GetConfig().map_node_size_y_;
  _resolution = _map.GetConfig().map_resolutions_[resolution_id];

  _lidar_map_node = new MapNodeData(_node_size_x, _node_size_y);

  _search_range_x = search_range_x;
  _search_range_y = search_range_y;
  _lidar_locator.Init(_search_range_x, _search_range_y,
                      _resolution, _node_size_x, _node_size_y);
  return true;
}

void LocalizationLidar::SetVelodyneExtrinsic(const Eigen::Affine3d& pose) {
  // (double x, double y, double z, double qx, double qy, double qz, double qw)
  _velodyne_extrinsic = pose;
  Eigen::Vector3d trans = pose.translation();
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose.linear());

  _lidar_locator.SetVelodyneExtrinsic(trans.x(), trans.y(), trans.z(),
      quat.x(), quat.y(), quat.z(), quat.w());
  return;
}

void LocalizationLidar::SetVehicleHeight(double height) {
  // _lidar_locator.SetVehicleHeight(height);
  _vehicle_lidar_height = height;
  std::cout << "Set height: " << _vehicle_lidar_height;
  return;
}

void LocalizationLidar::SetValidThreshold(float valid_threashold) {
  _lidar_locator.SetValidThreshold(valid_threashold);
  return;
}

void LocalizationLidar::SetImageAlignMode(int mode) {
  _lidar_locator.SetImageAlignMode(mode);
  return;
}

void LocalizationLidar::SetLocalizationMode(int mode) {
  _lidar_locator.SetLocalizationMode(mode);
  return;
}

void LocalizationLidar::SetDeltaYawLimit(double limit) {
  _lidar_locator.SetDeltaYawLimit(limit);
}

void LocalizationLidar::SetDeltaPitchRollLimit(double limit) {
  _lidar_locator.SetDeltaPitchRollLimit(limit);
}

int LocalizationLidar::Update(unsigned int frame_idx,
                              const Eigen::Affine3d& pose,
                              const Eigen::Vector3d velocity,
                              const LidarFrame& lidar_frame) {
  // check whether loaded map
  if (_is_map_loaded == false) {
    _map.LoadMapArea(pose.translation(), _resolution_id,
                     _zone_id, _search_range_x, _search_range_y);
    _is_map_loaded = true;
    LOG(INFO) << "Reflectance locator map first loading is done.";
  }

  Eigen::Affine3d imu_pose = pose;
  RefineAltitudeFromMap(&imu_pose);

  // load all needed map
  Eigen::Vector3d pose_trans = imu_pose.translation();
  Eigen::Quaterniond pose_quat(imu_pose.linear());
  pose_quat.normalize();
  _map.LoadMapArea(pose_trans, _resolution_id,
                   _zone_id, 0, 0);

  // preload map for next locate
  _map.PreloadMapArea(pose_trans, velocity, _resolution_id,
                      _zone_id);

  // generate composed map for compare
  ComposeMapNode(pose_trans);

  // pass map node to locator
  int node_width = _lidar_map_node->width;
  int node_height = _lidar_map_node->height;
  int node_level_num = 1;
  _lidar_locator.SetMapNodeData(node_width, node_height, node_level_num,
      &(_lidar_map_node->intensities), &(_lidar_map_node->intensities_var),
      &(_lidar_map_node->altitudes), &(_lidar_map_node->count));
  _lidar_locator.SetMapNodeLeftTopCorner(_map_left_top_corner(0),
                                         _map_left_top_corner(1));

  // pass lidar points to locator
  int size = lidar_frame.pt_xs.size();
  _lidar_locator.SetPointCloudData(
      size, lidar_frame.pt_xs.data(), lidar_frame.pt_ys.data(),
      lidar_frame.pt_zs.data(), lidar_frame.intensities.data());

  // compute
  int error = _lidar_locator.Compute(pose_trans(0),
                                     pose_trans(1),
                                     pose_trans(2),
                                     pose_quat.x(),
                                     pose_quat.y(),
                                     pose_quat.z(),
                                     pose_quat.w());

  return error;
}

void LocalizationLidar::GetResult(Eigen::Affine3d *location,
                                  Eigen::Matrix3d *covariance) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  _lidar_locator.GetLocationPose(&x, &y, &z, &qx, &qy, &qz, &qw);

  Eigen::Quaterniond quatd(qw, qx, qy, qz);
  Eigen::Translation3d transd(Eigen::Vector3d(x, y, z));
  *location = transd * quatd;

  RefineAltitudeFromMap(location);

  int width = 0;
  int height = 0;
  const double* data = NULL;
  _lidar_locator.GetLocationCovariance(&data, &width, &height);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      (*covariance)(row, col) = data[row * 3 + col];
    }
  }

  return;
}

void LocalizationLidar::GetLocalizationDistribution(
    Eigen::MatrixXd *distribution) {
  int width = 0;
  int height = 0;
  const double *data = NULL;
  _lidar_locator.GetSSDDistribution(&data, &width, &height);

  *distribution = Eigen::MatrixXd::Zero(height, width);
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      (*distribution)(row, col) = data[row * width + col];
    }
  }
  return;
}

void LocalizationLidar::RefineAltitudeFromMap(Eigen::Affine3d *pose) {
  Eigen::Affine3d lidar_pose = *pose * _velodyne_extrinsic;
  Eigen::Vector3d lidar_trans = lidar_pose.translation();

  // Read the altitude from the map
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
      _map.GetConfig(), lidar_trans, _resolution_id, _zone_id);
  LossyMapNode* node =
      static_cast<LossyMapNode*>(_map.GetMapNodeSafe(index));
  LossyMapMatrix& matrix =
      static_cast<LossyMapMatrix&>(node->GetMapCellMatrix());
  unsigned int x = 0;
  unsigned int y = 0;
  node->GetCoordinate(lidar_trans, &x, &y);

  const double height_diff = _vehicle_lidar_height;

  if (_is_pre_ground_height_valid == false) {
    _is_pre_ground_height_valid = true;
    _pre_vehicle_ground_height = lidar_pose.translation()(2) - height_diff;
  }

  float vehicle_ground_alt = 0.0;
  unsigned int count = matrix[y][x].count;
  if (count > 0) {
    if (matrix[y][x].is_ground_useful) {
      vehicle_ground_alt = matrix[y][x].altitude_ground;
    } else {
      vehicle_ground_alt = matrix[y][x].altitude;
    }
  } else {
    vehicle_ground_alt = _pre_vehicle_ground_height;
  }

  lidar_pose.translation()(2) = vehicle_ground_alt + height_diff;
  Eigen::Affine3d transform_tmp
        = lidar_pose * _velodyne_extrinsic.inverse();
  pose->translation() = transform_tmp.translation();
  return;
}

void LocalizationLidar::ComposeMapNode(const Eigen::Vector3d& trans) {
  Eigen::Vector2d center(trans(0), trans(1));
  Eigen::Vector2d left_top_corner(
      center(0) - _node_size_x * _resolution / 2.0,
      center(1) - _node_size_y * _resolution / 2.0);

  // get map node index 2x2
  MapNodeIndex map_node_idx[2][2];
  // top left corner
  map_node_idx[0][0] = MapNodeIndex::GetMapNodeIndex(
      _map.GetConfig(), left_top_corner, _resolution_id, _zone_id);
  // top right corner
  map_node_idx[0][1] = map_node_idx[0][0];
  map_node_idx[0][1].n_ += 1;
  // bottom left corner
  map_node_idx[1][0] = map_node_idx[0][0];
  map_node_idx[1][0].m_ += 1;
  // bottom right corner
  map_node_idx[1][1] = map_node_idx[0][0];
  map_node_idx[1][1].n_ += 1;
  map_node_idx[1][1].m_ += 1;

  // get map node 2x2
  LossyMapNode* map_node[2][2] = {NULL};
  for (unsigned int y = 0; y < 2; ++y) {
    for (unsigned int x = 0; x < 2; ++x) {
      map_node[y][x] = static_cast<LossyMapNode*>(
          _map.GetMapNodeSafe(map_node_idx[y][x]));
    }
  }

  // compose map node
  unsigned int coord_x = 0;
  unsigned int coord_y = 0;
  map_node[0][0]->GetCoordinate(
      left_top_corner, &coord_x, &coord_y);
  _map_left_top_corner = map_node[0][0]->GetCoordinate(coord_x, coord_y);

  int coord_xi = coord_x;
  int coord_yi = coord_y;
  int range_xs[2][2] = {0};
  int range_ys[2][2] = {0};
  range_xs[0][0] = _node_size_x - coord_xi;
  range_xs[1][0] = _node_size_x - coord_xi;
  range_xs[0][1] = coord_xi;
  range_xs[1][1] = coord_xi;
  range_ys[0][0] = _node_size_y - coord_yi;
  range_ys[0][1] = _node_size_y - coord_yi;
  range_ys[1][0] = coord_yi;
  range_ys[1][1] = coord_yi;

  int src_xs[2][2] = {0};
  int src_ys[2][2] = {0};
  src_xs[0][0] = coord_xi;
  src_xs[1][0] = coord_xi;
  src_xs[0][1] = 0;
  src_xs[1][1] = 0;
  src_ys[0][0] = coord_yi;
  src_ys[0][1] = coord_yi;
  src_ys[1][0] = 0;
  src_ys[1][1] = 0;

  int dst_xs[2][2] = {0};
  int dst_ys[2][2] = {0};
  dst_xs[0][0] = 0;
  dst_xs[1][0] = 0;
  dst_xs[0][1] = _node_size_x - coord_xi;
  dst_xs[1][1] = _node_size_x - coord_xi;
  dst_ys[0][0] = 0;
  dst_ys[0][1] = 0;
  dst_ys[1][0] = _node_size_y - coord_yi;
  dst_ys[1][1] = _node_size_y - coord_yi;

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      int range_x = range_xs[i][j];
      int range_y = range_ys[i][j];
      int src_x = src_xs[i][j];
      int src_y = src_ys[i][j];
      int dst_x = dst_xs[i][j];
      int dst_y = dst_ys[i][j];
      LossyMapMatrix& map_cells =
          static_cast<LossyMapMatrix&>(map_node[i][j]->GetMapCellMatrix());
      for (int y = 0; y < range_y; ++y) {
        int dst_base_x = (dst_y + y) * _node_size_x + dst_x;
        for (int x = 0; x < range_x; ++x) {
          auto &cell = map_cells[src_y + y][src_x + x];
          int dst_idx = dst_base_x + x;
          _lidar_map_node->intensities[dst_idx] = cell.intensity;
          _lidar_map_node->intensities_var[dst_idx] = cell.intensity_var;
          _lidar_map_node->altitudes[dst_idx] = cell.altitude;
          _lidar_map_node->count[dst_idx] = cell.count;
        }
      }
    }
  }
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
