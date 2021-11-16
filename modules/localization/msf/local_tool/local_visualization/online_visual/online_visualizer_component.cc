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

#include "modules/localization/msf/local_tool/local_visualization/online_visual/online_visualizer_component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"

#include "cyber/time/clock.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

OnlineVisualizerComponent::OnlineVisualizerComponent() {}

OnlineVisualizerComponent::~OnlineVisualizerComponent() {
  VisualizationManager::GetInstance().StopVisualization();
}

bool OnlineVisualizerComponent::Init() {
  if (!InitConfig()) {
    AERROR << "InitParams failed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "InitIO failed.";
    return false;
  }

  VisualizationManager::GetInstance().StartVisualization();

  return true;
}

bool OnlineVisualizerComponent::InitConfig() {
  map_folder_ = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  map_visual_folder_ = FLAGS_map_visual_dir;
  lidar_extrinsic_file_ = FLAGS_lidar_extrinsics_file;

  lidar_local_topic_ = FLAGS_localization_lidar_topic;
  gnss_local_topic_ = FLAGS_localization_gnss_topic;
  fusion_local_topic_ = FLAGS_localization_topic;
  ndt_local_topic_ = FLAGS_localization_ndt_topic;

  Eigen::Affine3d velodyne_extrinsic;
  bool success =
      velodyne::LoadExtrinsic(lidar_extrinsic_file_, &velodyne_extrinsic);
  if (!success) {
    AERROR << "Load lidar extrinsic failed." << std::endl;
    return false;
  }
  std::cout << "Load lidar extrinsic succeed." << std::endl;

  pyramid_map::BaseMapConfig map_config;
  std::string config_file = map_folder_ + "/config.xml";
  map_config.map_version_ = "lossy_map";
  success = map_config.Load(config_file);
  if (!success) {
    AERROR << "Load map config failed." << std::endl;
    return false;
  }
  std::cout << "Load map config succeed." << std::endl;

  VisualMapParam map_param;
  map_param.set(map_config.map_resolutions_, map_config.map_node_size_x_,
                map_config.map_node_size_y_, map_config.map_range_.GetMinX(),
                map_config.map_range_.GetMinY(),
                map_config.map_range_.GetMaxX(),
                map_config.map_range_.GetMaxY());
  success = VisualizationManager::GetInstance().Init(
      map_folder_, map_visual_folder_, velodyne_extrinsic, map_param);
  if (!success) {
    return false;
  }
  return true;
}

bool OnlineVisualizerComponent::InitIO() {
  // Lidar localization
  std::function<void(const std::shared_ptr<LocalizationEstimate> &)>
      lidar_local_call =
          std::bind(&OnlineVisualizerComponent::OnLidarLocalization, this,
                    std::placeholders::_1);
  lidar_local_listener_ = this->node_->CreateReader<LocalizationEstimate>(
      lidar_local_topic_, lidar_local_call);

  // GNSS localization
  std::function<void(const std::shared_ptr<LocalizationEstimate> &)>
      gnss_local_call =
          std::bind(&OnlineVisualizerComponent::OnGNSSLocalization, this,
                    std::placeholders::_1);
  gnss_local_listener_ = this->node_->CreateReader<LocalizationEstimate>(
      gnss_local_topic_, gnss_local_call);

  // Fusion localization
  std::function<void(const std::shared_ptr<LocalizationEstimate> &)>
      fusion_local_call =
          std::bind(&OnlineVisualizerComponent::OnFusionLocalization, this,
                    std::placeholders::_1);
  fusion_local_listener_ = this->node_->CreateReader<LocalizationEstimate>(
      fusion_local_topic_, fusion_local_call);

  // Ndt Localization
  std::function<void(const std::shared_ptr<LocalizationEstimate> &)>
      ndt_local_call =
          std::bind(&OnlineVisualizerComponent::OnLidarLocalization, this,
                    std::placeholders::_1);
  lidar_local_listener_ = this->node_->CreateReader<LocalizationEstimate>(
      ndt_local_topic_, ndt_local_call);

  return true;
}

bool OnlineVisualizerComponent::Proc(
    const std::shared_ptr<drivers::PointCloud> &msg) {
  LidarVisFrame lidar_vis_frame;
  lidar_vis_frame.timestamp = cyber::Time(msg->measurement_time()).ToSecond();

  std::vector<unsigned char> intensities;
  ParsePointCloudMessage(msg, &lidar_vis_frame.pt3ds, &intensities);

  static unsigned int id = 1;
  lidar_vis_frame.frame_id = id;
  VisualizationManager::GetInstance().AddLidarFrame(lidar_vis_frame);

  id++;
  return true;
}

void OnlineVisualizerComponent::OnLidarLocalization(
    const std::shared_ptr<LocalizationEstimate> &msg) {
  LocalizationMsg lidar_loc_msg;

  lidar_loc_msg.timestamp = msg->measurement_time();
  lidar_loc_msg.x = msg->pose().position().x();
  lidar_loc_msg.y = msg->pose().position().y();
  lidar_loc_msg.z = msg->pose().position().z();

  lidar_loc_msg.qx = msg->pose().orientation().qx();
  lidar_loc_msg.qy = msg->pose().orientation().qy();
  lidar_loc_msg.qz = msg->pose().orientation().qz();
  lidar_loc_msg.qw = msg->pose().orientation().qw();

  if (msg->has_uncertainty() &&
      !std::isnan(msg->uncertainty().position_std_dev().x()) &&
      !std::isnan(msg->uncertainty().position_std_dev().y()) &&
      msg->uncertainty().position_std_dev().x() > 0 &&
      msg->uncertainty().position_std_dev().y() > 0) {
    lidar_loc_msg.std_x = msg->uncertainty().position_std_dev().x();
    lidar_loc_msg.std_y = msg->uncertainty().position_std_dev().y();
    lidar_loc_msg.std_z = msg->uncertainty().position_std_dev().z();
  }

  VisualizationManager::GetInstance().AddLidarLocMessage(lidar_loc_msg);
}

void OnlineVisualizerComponent::OnGNSSLocalization(
    const std::shared_ptr<LocalizationEstimate> &msg) {
  LocalizationMsg gnss_loc_msg;

  gnss_loc_msg.timestamp = msg->measurement_time();
  gnss_loc_msg.x = msg->pose().position().x();
  gnss_loc_msg.y = msg->pose().position().y();
  gnss_loc_msg.z = msg->pose().position().z();

  gnss_loc_msg.qx = msg->pose().orientation().qx();
  gnss_loc_msg.qy = msg->pose().orientation().qy();
  gnss_loc_msg.qz = msg->pose().orientation().qz();
  gnss_loc_msg.qw = msg->pose().orientation().qw();

  if (msg->has_uncertainty() &&
      !std::isnan(msg->uncertainty().position_std_dev().x()) &&
      !std::isnan(msg->uncertainty().position_std_dev().y()) &&
      msg->uncertainty().position_std_dev().x() > 0 &&
      msg->uncertainty().position_std_dev().y() > 0) {
    gnss_loc_msg.std_x = msg->uncertainty().position_std_dev().x();
    gnss_loc_msg.std_y = msg->uncertainty().position_std_dev().y();
    gnss_loc_msg.std_z = msg->uncertainty().position_std_dev().z();
  }

  VisualizationManager::GetInstance().AddGNSSLocMessage(gnss_loc_msg);
}

void OnlineVisualizerComponent::OnFusionLocalization(
    const std::shared_ptr<LocalizationEstimate> &msg) {
  LocalizationMsg fusion_loc_msg;

  fusion_loc_msg.timestamp = msg->measurement_time();
  fusion_loc_msg.x = msg->pose().position().x();
  fusion_loc_msg.y = msg->pose().position().y();
  fusion_loc_msg.z = msg->pose().position().z();

  fusion_loc_msg.qx = msg->pose().orientation().qx();
  fusion_loc_msg.qy = msg->pose().orientation().qy();
  fusion_loc_msg.qz = msg->pose().orientation().qz();
  fusion_loc_msg.qw = msg->pose().orientation().qw();

  if (msg->has_uncertainty() &&
      !std::isnan(msg->uncertainty().position_std_dev().x()) &&
      !std::isnan(msg->uncertainty().position_std_dev().y()) &&
      msg->uncertainty().position_std_dev().x() > 0 &&
      msg->uncertainty().position_std_dev().y() > 0) {
    fusion_loc_msg.std_x = msg->uncertainty().position_std_dev().x();
    fusion_loc_msg.std_y = msg->uncertainty().position_std_dev().y();
    fusion_loc_msg.std_z = msg->uncertainty().position_std_dev().z();
  }

  VisualizationManager::GetInstance().AddFusionLocMessage(fusion_loc_msg);
}

void OnlineVisualizerComponent::ParsePointCloudMessage(
    const std::shared_ptr<drivers::PointCloud> &msg,
    ::apollo::common::EigenVector3dVec *pt3ds,
    std::vector<unsigned char> *intensities) {
  CHECK_NOTNULL(pt3ds);
  CHECK_NOTNULL(intensities);

  if (msg->height() > 1 && msg->width() > 1) {
    for (unsigned int i = 0; i < msg->height(); ++i) {
      for (unsigned int j = 0; j < msg->width(); ++j) {
        Eigen::Vector3d pt3d;
        pt3d[0] = static_cast<double>(msg->point(i * msg->width() + j).x());
        pt3d[1] = static_cast<double>(msg->point(i * msg->width() + j).y());
        pt3d[2] = static_cast<double>(msg->point(i * msg->width() + j).z());
        if (!std::isnan(pt3d[0])) {
          unsigned char intensity = static_cast<unsigned char>(
              msg->point(i * msg->width() + j).intensity());
          pt3ds->push_back(pt3d);
          intensities->push_back(intensity);
        }
      }
    }
  } else {
    AINFO << "Receiving un-organized-point-cloud, width " << msg->width()
          << " height " << msg->height() << "size " << msg->point_size();
    for (int i = 0; i < msg->point_size(); ++i) {
      Eigen::Vector3d pt3d;
      pt3d[0] = static_cast<double>(msg->point(i).x());
      pt3d[1] = static_cast<double>(msg->point(i).y());
      pt3d[2] = static_cast<double>(msg->point(i).z());
      if (!std::isnan(pt3d[0])) {
        unsigned char intensity =
            static_cast<unsigned char>(msg->point(i).intensity());
        pt3ds->push_back(pt3d);
        intensities->push_back(intensity);
      }
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
