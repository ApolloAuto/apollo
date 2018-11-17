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

#include "modules/localization/msf/local_integ/localization_lidar.h"

namespace apollo {
namespace localization {
namespace msf {

LocalizationLidar::LocalizationLidar()
    : lidar_locator_(new LidarLocator()),
      search_range_x_(21), search_range_y_(21),
      node_size_x_(1024), node_size_y_(1024),
      resolution_(0.125), lidar_map_node_(nullptr),
      config_("lossy_map"), map_(&config_),
      map_node_pool_(25, 8),
      resolution_id_(0),
      is_map_loaded_(false),
      vehicle_lidar_height_(1.7),
      pre_vehicle_ground_height_(0.0),
      is_pre_ground_height_valid_(false),
      velodyne_extrinsic_(Eigen::Affine3d::Identity()) {
  map_left_top_corner_ = Eigen::Vector2d::Zero();
}

LocalizationLidar::~LocalizationLidar() {
  if (lidar_map_node_) {
    delete lidar_map_node_;
    lidar_map_node_ = nullptr;
  }

  delete lidar_locator_;
  lidar_locator_ = nullptr;
}

bool LocalizationLidar::Init(const std::string& map_path,
                             const unsigned int search_range_x,
                             const unsigned int search_range_y,
                             const int zone_id,
                             const unsigned int resolution_id) {
  // init map
  resolution_id_ = resolution_id;
  zone_id_ = zone_id;
  if (!map_.SetMapFolderPath(map_path)) {
    LOG(FATAL) << "Reflectance map folder is invalid!";
    return false;
  }
  map_node_pool_.Initial(&(map_.GetConfig()));
  map_.InitThreadPool(1, 6);
  map_.InitMapNodeCaches(12, 24);
  map_.AttachMapNodePool(&map_node_pool_);

  // init locator
  node_size_x_ = map_.GetConfig().map_node_size_x_;
  node_size_y_ = map_.GetConfig().map_node_size_y_;
  resolution_ = map_.GetConfig().map_resolutions_[resolution_id];

  lidar_map_node_ = new MapNodeData(node_size_x_, node_size_y_);

  search_range_x_ = search_range_x;
  search_range_y_ = search_range_y;
  lidar_locator_->Init(search_range_x_, search_range_y_,
                      resolution_, node_size_x_, node_size_y_);
  return true;
}

void LocalizationLidar::SetVelodyneExtrinsic(const Eigen::Affine3d& pose) {
  velodyne_extrinsic_ = pose;
  Eigen::Vector3d trans = pose.translation();
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose.linear());

  lidar_locator_->SetVelodyneExtrinsic(trans.x(), trans.y(), trans.z(),
      quat.x(), quat.y(), quat.z(), quat.w());
  return;
}

void LocalizationLidar::SetVehicleHeight(double height) {
  vehicle_lidar_height_ = height;
  AINFO << "Set height: " << vehicle_lidar_height_;
  return;
}

void LocalizationLidar::SetValidThreshold(float valid_threashold) {
  lidar_locator_->SetValidThreshold(valid_threashold);
  return;
}

void LocalizationLidar::SetImageAlignMode(int mode) {
  lidar_locator_->SetImageAlignMode(mode);
  return;
}

void LocalizationLidar::SetLocalizationMode(int mode) {
  lidar_locator_->SetLocalizationMode(mode);
  return;
}

void LocalizationLidar::SetDeltaYawLimit(double limit) {
  lidar_locator_->SetDeltaYawLimit(limit);
}

void LocalizationLidar::SetDeltaPitchRollLimit(double limit) {
  lidar_locator_->SetDeltaPitchRollLimit(limit);
}

int LocalizationLidar::Update(const unsigned int frame_idx,
                              const Eigen::Affine3d& pose,
                              const Eigen::Vector3d velocity,
                              const LidarFrame& lidar_frame) {
  // check whether loaded map
  if (is_map_loaded_ == false) {
    map_.LoadMapArea(pose.translation(), resolution_id_,
                     zone_id_, search_range_x_, search_range_y_);
    is_map_loaded_ = true;
    AINFO << "Reflectance locator map first loading is done.";
  }

  Eigen::Affine3d imu_pose = pose;
  RefineAltitudeFromMap(&imu_pose);

  // load all needed map
  Eigen::Vector3d pose_trans = imu_pose.translation();
  Eigen::Quaterniond pose_quat(imu_pose.linear());
  pose_quat.normalize();
  map_.LoadMapArea(pose_trans, resolution_id_,
                   zone_id_, 0, 0);

  // preload map for next locate
  map_.PreloadMapArea(pose_trans, velocity, resolution_id_,
                      zone_id_);

  // generate composed map for compare
  ComposeMapNode(pose_trans);

  // pass map node to locator
  int node_width = lidar_map_node_->width;
  int node_height = lidar_map_node_->height;
  int node_level_num = 1;
  lidar_locator_->SetMapNodeData(node_width, node_height, node_level_num,
      &(lidar_map_node_->intensities), &(lidar_map_node_->intensities_var),
      &(lidar_map_node_->altitudes), &(lidar_map_node_->count));
  lidar_locator_->SetMapNodeLeftTopCorner(map_left_top_corner_(0),
                                          map_left_top_corner_(1));

  // pass lidar points to locator
  int size = lidar_frame.pt_xs.size();
  lidar_locator_->SetPointCloudData(
      size, lidar_frame.pt_xs.data(), lidar_frame.pt_ys.data(),
      lidar_frame.pt_zs.data(), lidar_frame.intensities.data());

  // compute
  int error = lidar_locator_->Compute(pose_trans(0),
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
  if (!location || !covariance) {
    return;
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  lidar_locator_->GetLocationPose(&x, &y, &z, &qx, &qy, &qz, &qw);

  Eigen::Quaterniond quatd(qw, qx, qy, qz);
  Eigen::Translation3d transd(Eigen::Vector3d(x, y, z));
  *location = transd * quatd;

  RefineAltitudeFromMap(location);

  int width = 0;
  int height = 0;
  const double* data = nullptr;
  lidar_locator_->GetLocationCovariance(&data, &width, &height);

  if (width >= 3 && height >= 3 && data != nullptr) {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        (*covariance)(row, col) = data[row * 3 + col];
      }
    }
  }
  return;
}

void LocalizationLidar::GetLocalizationDistribution(
    Eigen::MatrixXd *distribution) {
  CHECK_NOTNULL(distribution);

  int width = 0;
  int height = 0;
  const double *data = nullptr;
  lidar_locator_->GetSSDDistribution(&data, &width, &height);

  if (width > 0 && height > 0 && data != nullptr) {
    *distribution = Eigen::MatrixXd::Zero(height, width);
    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        (*distribution)(row, col) = data[row * width + col];
      }
    }
  }
  return;
}

void LocalizationLidar::RefineAltitudeFromMap(Eigen::Affine3d *pose) {
  CHECK_NOTNULL(pose);

  Eigen::Affine3d lidar_pose = *pose * velodyne_extrinsic_;
  Eigen::Vector3d lidar_trans = lidar_pose.translation();

  // Read the altitude from the map
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
      map_.GetConfig(), lidar_trans, resolution_id_, zone_id_);
  LossyMapNode* node =
      static_cast<LossyMapNode*>(map_.GetMapNodeSafe(index));
  LossyMapMatrix& matrix =
      static_cast<LossyMapMatrix&>(node->GetMapCellMatrix());

  const double height_diff = vehicle_lidar_height_;

  if (is_pre_ground_height_valid_ == false) {
    is_pre_ground_height_valid_ = true;
    pre_vehicle_ground_height_ = lidar_pose.translation()(2) - height_diff;
  }

  float vehicle_ground_alt = 0.0;
  unsigned int x = 0;
  unsigned int y = 0;
  if (node->GetCoordinate(lidar_trans, &x, &y)) {
    unsigned int count = matrix[y][x].count;
    if (count > 0) {
      if (matrix[y][x].is_ground_useful) {
        vehicle_ground_alt = matrix[y][x].altitude_ground;
      } else {
        vehicle_ground_alt = matrix[y][x].altitude;
      }
    } else {
      vehicle_ground_alt = pre_vehicle_ground_height_;
    }
  }

  lidar_pose.translation()(2) = vehicle_ground_alt + height_diff;
  Eigen::Affine3d transform_tmp
        = lidar_pose * velodyne_extrinsic_.inverse();
  pose->translation() = transform_tmp.translation();
  return;
}

void LocalizationLidar::ComposeMapNode(const Eigen::Vector3d& trans) {
  Eigen::Vector2d center(trans(0), trans(1));
  Eigen::Vector2d left_top_corner(
      center(0) - node_size_x_ * resolution_ / 2.0,
      center(1) - node_size_y_ * resolution_ / 2.0);

  // get map node index 2x2
  MapNodeIndex map_node_idx[2][2];
  // top left corner
  map_node_idx[0][0] = MapNodeIndex::GetMapNodeIndex(
      map_.GetConfig(), left_top_corner, resolution_id_, zone_id_);
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
  LossyMapNode* map_node[2][2] = {nullptr};
  for (unsigned int y = 0; y < 2; ++y) {
    for (unsigned int x = 0; x < 2; ++x) {
      map_node[y][x] = static_cast<LossyMapNode*>(
          map_.GetMapNodeSafe(map_node_idx[y][x]));
    }
  }

  // compose map node
  unsigned int coord_x = 0;
  unsigned int coord_y = 0;
  map_node[0][0]->GetCoordinate(
      left_top_corner, &coord_x, &coord_y);
  map_left_top_corner_ = map_node[0][0]->GetCoordinate(coord_x, coord_y);

  int coord_xi = coord_x;
  int coord_yi = coord_y;
  int range_xs[2][2] = {0};
  int range_ys[2][2] = {0};
  range_xs[0][0] = node_size_x_ - coord_xi;
  range_xs[1][0] = node_size_x_ - coord_xi;
  range_xs[0][1] = coord_xi;
  range_xs[1][1] = coord_xi;
  range_ys[0][0] = node_size_y_ - coord_yi;
  range_ys[0][1] = node_size_y_ - coord_yi;
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
  dst_xs[0][1] = node_size_x_ - coord_xi;
  dst_xs[1][1] = node_size_x_ - coord_xi;
  dst_ys[0][0] = 0;
  dst_ys[0][1] = 0;
  dst_ys[1][0] = node_size_y_ - coord_yi;
  dst_ys[1][1] = node_size_y_ - coord_yi;

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
        int dst_base_x = (dst_y + y) * node_size_x_ + dst_x;
        for (int x = 0; x < range_x; ++x) {
          auto &cell = map_cells[src_y + y][src_x + x];
          int dst_idx = dst_base_x + x;
          lidar_map_node_->intensities[dst_idx] = cell.intensity;
          lidar_map_node_->intensities_var[dst_idx] = cell.intensity_var;
          lidar_map_node_->altitudes[dst_idx] = cell.altitude;
          lidar_map_node_->count[dst_idx] = cell.count;
        }
      }
    }
  }
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
