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

#include "modules/localization/msf/local_tool/local_visualization/online_visual/online_local_visualizer.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/local_map/base_map/base_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

using ::Eigen::Vector3d;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::time::Clock;

OnlineLocalVisualizer::OnlineLocalVisualizer()
    : monitor_logger_(MonitorMessageItem::LOCALIZATION) {}

OnlineLocalVisualizer::~OnlineLocalVisualizer() {}

std::string OnlineLocalVisualizer::Name() const {
  return "localization_visualization";
}

Status OnlineLocalVisualizer::Start() {
  AdapterManager::Init(FLAGS_msf_visual_adapter_config_file);

  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  // Point Cloud
  if (!AdapterManager::GetPointCloud()) {
    buffer.ERROR(
        "PointCloud input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR_MSG, "no PointCloud adapter");
  }
  AdapterManager::AddPointCloudCallback(&OnlineLocalVisualizer::OnPointCloud,
                                        this);

  // Lidar localization
  if (!AdapterManager::GetLocalizationMsfLidar()) {
    buffer.ERROR(
        "LocalizationMsfLidar input not initialized. Check your adapter.conf "
        "file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR_MSG,
                  "no LocalizationMsfLidar adapter");
  }
  AdapterManager::AddLocalizationMsfLidarCallback(
      &OnlineLocalVisualizer::OnLidarLocalization, this);

  // GNSS localization
  if (!AdapterManager::GetLocalizationMsfGnss()) {
    buffer.ERROR(
        "LocalizationMsfGnss input not initialized. Check your adapter.conf "
        "file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR_MSG,
                  "no LocalizationMsfGnss adapter");
  }
  AdapterManager::AddLocalizationMsfGnssCallback(
      &OnlineLocalVisualizer::OnGNSSLocalization, this);

  // Fusion localization
  if (!AdapterManager::GetLocalization()) {
    buffer.ERROR(
        "Localization input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR_MSG, "no Localization adapter");
  }
  AdapterManager::AddLocalizationCallback(
      &OnlineLocalVisualizer::OnFusionLocalization, this);

  VisualizationManager::GetInstance().StartVisualization();

  return Status::OK();
}

void OnlineLocalVisualizer::Stop() {
  VisualizationManager::GetInstance().StopVisualization();
}

Status OnlineLocalVisualizer::Init() {
  InitParams();

  Eigen::Affine3d velodyne_extrinsic;
  bool success =
      velodyne::LoadExtrinsic(lidar_extrinsic_file_, &velodyne_extrinsic);
  if (!success) {
    std::cerr << "Load lidar extrinsic failed." << std::endl;
    return Status(common::LOCALIZATION_ERROR, "load lidar extrinsic failed");
  }
  std::cout << "Load lidar extrinsic succeed." << std::endl;

  BaseMapConfig map_config;
  std::string config_file = map_folder_ + "/config.xml";
  map_config.map_version_ = "lossy_map";
  success = map_config.Load(config_file);
  if (!success) {
    std::cerr << "Load map config failed." << std::endl;
    return Status(common::LOCALIZATION_ERROR, "load map config failed");
  }
  std::cout << "Load map config succeed." << std::endl;

  VisualMapParam map_param;
  map_param.set(map_config.map_resolutions_, map_config.map_node_size_x_,
                map_config.map_node_size_y_, map_config.map_range_.GetMinX(),
                map_config.map_range_.GetMinY(),
                map_config.map_range_.GetMaxX(),
                map_config.map_range_.GetMaxY());
  success = VisualizationManager::GetInstance().Init(
      map_folder_, velodyne_extrinsic, map_param);
  if (!success) {
    return Status(common::LOCALIZATION_ERROR, "visualization init failed");
  }

  return Status::OK();
}

void OnlineLocalVisualizer::InitParams() {
  map_folder_ = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  lidar_extrinsic_file_ = FLAGS_lidar_extrinsics_file;
}

void OnlineLocalVisualizer::OnPointCloud(
    const sensor_msgs::PointCloud2 &message) {
  LidarVisFrame lidar_vis_frame;
  lidar_vis_frame.timestamp = message.header.stamp.toSec();

  std::vector<unsigned char> intensities;
  ParsePointCloudMessage(message, &lidar_vis_frame.pt3ds, &intensities);

  static unsigned int id = 1;
  lidar_vis_frame.frame_id = id;
  VisualizationManager::GetInstance().AddLidarFrame(lidar_vis_frame);

  id++;
}

void OnlineLocalVisualizer::OnLidarLocalization(
    const LocalizationEstimate &message) {
  LocalizationMsg lidar_loc_msg;

  lidar_loc_msg.timestamp = message.measurement_time();
  lidar_loc_msg.x = message.pose().position().x();
  lidar_loc_msg.y = message.pose().position().y();
  lidar_loc_msg.z = message.pose().position().z();

  lidar_loc_msg.qx = message.pose().orientation().qx();
  lidar_loc_msg.qy = message.pose().orientation().qy();
  lidar_loc_msg.qz = message.pose().orientation().qz();
  lidar_loc_msg.qw = message.pose().orientation().qw();

  if (message.has_uncertainty()) {
    lidar_loc_msg.std_x = message.uncertainty().position_std_dev().x();
    lidar_loc_msg.std_y = message.uncertainty().position_std_dev().y();
    lidar_loc_msg.std_z = message.uncertainty().position_std_dev().z();
  }

  VisualizationManager::GetInstance().AddLidarLocMessage(lidar_loc_msg);
}

void OnlineLocalVisualizer::OnGNSSLocalization(
    const LocalizationEstimate &message) {
  LocalizationMsg gnss_loc_msg;

  gnss_loc_msg.timestamp = message.measurement_time();
  gnss_loc_msg.x = message.pose().position().x();
  gnss_loc_msg.y = message.pose().position().y();
  gnss_loc_msg.z = message.pose().position().z();

  gnss_loc_msg.qx = message.pose().orientation().qx();
  gnss_loc_msg.qy = message.pose().orientation().qy();
  gnss_loc_msg.qz = message.pose().orientation().qz();
  gnss_loc_msg.qw = message.pose().orientation().qw();

  if (message.has_uncertainty()) {
    gnss_loc_msg.std_x = message.uncertainty().position_std_dev().x();
    gnss_loc_msg.std_y = message.uncertainty().position_std_dev().y();
    gnss_loc_msg.std_z = message.uncertainty().position_std_dev().z();
  }

  VisualizationManager::GetInstance().AddGNSSLocMessage(gnss_loc_msg);
}

void OnlineLocalVisualizer::OnFusionLocalization(
    const LocalizationEstimate &message) {
  LocalizationMsg fusion_loc_msg;

  fusion_loc_msg.timestamp = message.measurement_time();
  fusion_loc_msg.x = message.pose().position().x();
  fusion_loc_msg.y = message.pose().position().y();
  fusion_loc_msg.z = message.pose().position().z();

  fusion_loc_msg.qx = message.pose().orientation().qx();
  fusion_loc_msg.qy = message.pose().orientation().qy();
  fusion_loc_msg.qz = message.pose().orientation().qz();
  fusion_loc_msg.qw = message.pose().orientation().qw();

  if (message.has_uncertainty()) {
    fusion_loc_msg.std_x = message.uncertainty().position_std_dev().x();
    fusion_loc_msg.std_y = message.uncertainty().position_std_dev().y();
    fusion_loc_msg.std_z = message.uncertainty().position_std_dev().z();
  }

  VisualizationManager::GetInstance().AddFusionLocMessage(fusion_loc_msg);
}

void OnlineLocalVisualizer::ParsePointCloudMessage(
    const sensor_msgs::PointCloud2 &message,
    std::vector<Eigen::Vector3d> *pt3ds,
    std::vector<unsigned char> *intensities) {
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(message, pcl_cloud);

  pcl::PointCloud<velodyne::PointXYZIT>::Ptr cloud(
      new pcl::PointCloud<velodyne::PointXYZIT>);
  pcl::fromPCLPointCloud2(pcl_cloud, *cloud);

  if (cloud->height == 1 || cloud->width == 1) {
    std::cerr << "Un-organized-point-cloud" << std::endl;
    for (unsigned int i = 0; i < cloud->size(); ++i) {
      Eigen::Vector3d pt3d;
      pt3d[0] = (*cloud)[i].x;
      pt3d[1] = (*cloud)[i].y;
      pt3d[2] = (*cloud)[i].z;

      if (pt3d[0] == pt3d[0] && pt3d[1] == pt3d[1] && pt3d[2] == pt3d[2]) {
        Eigen::Vector3d pt3d_local = pt3d;
        unsigned char intensity =
            static_cast<unsigned char>((*cloud)[i].intensity);
        pt3ds->push_back(pt3d_local);
        intensities->push_back(intensity);
      }
    }
  } else {
    for (unsigned int h = 0; h < cloud->height; ++h) {
      for (unsigned int w = 0; w < cloud->width; ++w) {
        double x = cloud->at(w, h).x;
        double y = cloud->at(w, h).y;
        double z = cloud->at(w, h).z;
        Eigen::Vector3d pt3d(x, y, z);
        if (pt3d[0] == pt3d[0] && pt3d[1] == pt3d[1] && pt3d[2] == pt3d[2]) {
          Eigen::Vector3d pt3d_local = pt3d;
          unsigned char intensity =
              static_cast<unsigned char>(cloud->at(w, h).intensity);
          pt3ds->push_back(pt3d_local);
          intensities->push_back(intensity);
        }
      }
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
