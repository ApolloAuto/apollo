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

#include "modules/localization/msf/msf_localization.h"

#include <list>

#include "modules/drivers/gnss/proto/config.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using ::Eigen::Vector3d;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::time::Clock;

MSFLocalization::MSFLocalization()
    : monitor_(MonitorMessageItem::LOCALIZATION),
      tf2_broadcaster_(NULL),
      localization_state_(LocalizationMeasureState::OK),
      pcd_msg_index_(-1) {}

MSFLocalization::~MSFLocalization() {
  if (tf2_broadcaster_) {
    delete tf2_broadcaster_;
    tf2_broadcaster_ = NULL;
  }
}

Status MSFLocalization::Start() {
  AdapterManager::Init(FLAGS_msf_adapter_config_file);

  Status &&status = Init();
  if (!status.ok()) {
    return status;
  }

  common::monitor::MonitorBuffer buffer(&monitor_);

  // Raw Imu
  if (!AdapterManager::GetRawImu()) {
    buffer.ERROR(
        "Raw IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR_MSG, "no Raw IMU adapter");
  }
  AdapterManager::AddRawImuCallback(&MSFLocalization::OnRawImu, this);

  // Point Cloud
  if (FLAGS_enable_lidar_localization) {
    if (!AdapterManager::GetPointCloud()) {
      buffer.ERROR(
          "PointCloud input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no PointCloud adapter");
    }
    AdapterManager::AddPointCloudCallback(&MSFLocalization::OnPointCloud, this);
  }

  if (FLAGS_gnss_mode == 1) {
    // Gnss Rtk Obs
    if (!AdapterManager::GetGnssRtkObs()) {
      buffer.ERROR(
          "GnssRtkObs input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no GnssRtkObs adapter");
    }
    AdapterManager::AddGnssRtkObsCallback(&MSFLocalization::OnGnssRtkObs, this);

    // Gnss Rtk Eph
    if (!AdapterManager::GetGnssRtkEph()) {
      buffer.ERROR(
          "GnssRtkEph input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no GnssRtkEph adapter");
    }
    AdapterManager::AddGnssRtkEphCallback(&MSFLocalization::OnGnssRtkEph, this);
  } else {
    // Gnss Best Pose
    if (!AdapterManager::GetGnssBestPose()) {
      buffer.ERROR(
          "GnssBestPose input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no GnssBestPose adapter");
    }
    AdapterManager::AddGnssBestPoseCallback(&MSFLocalization::OnGnssBestPose,
                                            this);
  }

  tf2_broadcaster_ = new tf2_ros::TransformBroadcaster;

  return Status::OK();
}

Status MSFLocalization::Stop() {
  return Status::OK();
}

Status MSFLocalization::Init() {
  InitParams();

  LocalizationState &&state = localization_integ_.Init(localizaiton_param_);
  switch (state.error_code()) {
    case LocalizationErrorCode::INTEG_ERROR:
      return Status(common::LOCALIZATION_ERROR_INTEG, state.error_msg());
    case LocalizationErrorCode::LIDAR_ERROR:
      return Status(common::LOCALIZATION_ERROR_LIDAR, state.error_msg());
    case LocalizationErrorCode::GNSS_ERROR:
      return Status(common::LOCALIZATION_ERROR_GNSS, state.error_msg());
    default:
      return Status::OK();
  }
}

void MSFLocalization::InitParams() {
  // integration module
  localizaiton_param_.is_ins_can_self_align = FLAGS_integ_ins_can_self_align;
  localizaiton_param_.is_sins_align_with_vel = FLAGS_integ_sins_align_with_vel;
  localizaiton_param_.vel_threshold_get_yaw = FLAGS_vel_threshold_get_yaw;
  localizaiton_param_.integ_debug_log_flag = FLAGS_integ_debug_log_flag;
  localizaiton_param_.is_trans_gpstime_to_utctime =
      FLAGS_trans_gpstime_to_utctime;
  localizaiton_param_.gnss_mode = FLAGS_gnss_mode;
  localizaiton_param_.is_using_raw_gnsspos = true;

  // gnss module
  localizaiton_param_.enable_ins_aid_rtk = FLAGS_enable_ins_aid_rtk;
  localizaiton_param_.enable_auto_save_eph_file =
      FLAGS_enable_auto_save_eph_file;
  localizaiton_param_.eph_buffer_path = FLAGS_eph_buffer_path;

  // lidar module
  localizaiton_param_.map_path = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  // localizaiton_param_.lidar_extrinsic_file =
  //     common::util::TranslatePath(FLAGS_velodyne_params_target_path) +
  //     "/velodyne64_novatel_extrinsics_example.yaml";
  localizaiton_param_.lidar_extrinsic_file =
      "/home/tmp/ros/share/velodyne_pointcloud/params/"
      "velodyne64_novatel_extrinsics_example.yaml";
  localizaiton_param_.lidar_height_file =
      common::util::TranslatePath(FLAGS_lidar_height_file);
  localizaiton_param_.lidar_debug_log_flag = FLAGS_lidar_debug_log_flag;
  localizaiton_param_.localization_mode = FLAGS_lidar_localization_mode;
  localizaiton_param_.lidar_filter_size = FLAGS_lidar_filter_size;
  localizaiton_param_.lidar_thread_num = FLAGS_lidar_thread_num;
  localizaiton_param_.map_coverage_theshold = FLAGS_lidar_map_coverage_theshold;
  localizaiton_param_.imu_lidar_max_delay_time = FLAGS_lidar_imu_max_delay_time;

  AERROR << "map: " << localizaiton_param_.map_path;
  AERROR << "lidar_extrin: " << localizaiton_param_.lidar_extrinsic_file;
  AERROR << "lidar_height: " << localizaiton_param_.lidar_height_file;

  // common
  localizaiton_param_.utm_zone_id = FLAGS_local_utm_zone_id;
  localizaiton_param_.imu_rate = FLAGS_imu_rate;
  localizaiton_param_.enable_lidar_localization =
      FLAGS_enable_lidar_localization;

  localizaiton_param_.is_use_visualize = FLAGS_use_visualize;

  if (!FLAGS_imuant_from_gnss_conf_file) {
    localizaiton_param_.imu_to_ant_offset.offset_x = FLAGS_imu_to_ant_offset_x;
    localizaiton_param_.imu_to_ant_offset.offset_y = FLAGS_imu_to_ant_offset_y;
    localizaiton_param_.imu_to_ant_offset.offset_z = FLAGS_imu_to_ant_offset_z;
    localizaiton_param_.imu_to_ant_offset.uncertainty_x =
        FLAGS_imu_to_ant_offset_ux;
    localizaiton_param_.imu_to_ant_offset.uncertainty_y =
        FLAGS_imu_to_ant_offset_uy;
    localizaiton_param_.imu_to_ant_offset.uncertainty_z =
        FLAGS_imu_to_ant_offset_uz;
  } else {
    apollo::drivers::gnss::config::Config gnss_config;
    // CHECK(common::util::GetProtoFromASCIIFile<
    //       apollo::drivers::gnss::config::Config>(
    //     common::util::TranslatePath(FLAGS_gnss_conf_file_target_path),
    //     &gnss_config));
    CHECK_GT(gnss_config.login_commands_size(), 1);
    std::string login_commands = gnss_config.login_commands(1);
    bool found_imu_ant_parameter = false;
    for (int i = 0; i < gnss_config.login_commands_size(); ++i) {
      login_commands = gnss_config.login_commands(i);
      std::size_t found = login_commands.find(std::string("SETIMUTOANTOFFSET"));
      if (found != std::string::npos) {
        found_imu_ant_parameter = true;
        break;
      }
    }
    CHECK(found_imu_ant_parameter);

    std::vector<std::string> segmented_login_commands =
        common::util::StringTokenizer::Split(login_commands, " ");
    CHECK_EQ(segmented_login_commands.size(), 7);

    std::string name = "";
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    double uncertainty_x = 0.0;
    double uncertainty_y = 0.0;
    double uncertainty_z = 0.0;
    std::stringstream ss_str(login_commands);
    ss_str >> name >> offset_x >> offset_y >> offset_z >> uncertainty_x >>
        uncertainty_y >> uncertainty_z;
    localizaiton_param_.imu_to_ant_offset.offset_x = offset_x;
    localizaiton_param_.imu_to_ant_offset.offset_y = offset_y;
    localizaiton_param_.imu_to_ant_offset.offset_z = offset_z;
    localizaiton_param_.imu_to_ant_offset.uncertainty_x = uncertainty_x;
    localizaiton_param_.imu_to_ant_offset.uncertainty_y = uncertainty_y;
    localizaiton_param_.imu_to_ant_offset.uncertainty_z = uncertainty_z;

    AINFO << localizaiton_param_.imu_to_ant_offset.offset_x << " "
          << localizaiton_param_.imu_to_ant_offset.offset_y << " "
          << localizaiton_param_.imu_to_ant_offset.offset_z << " "
          << localizaiton_param_.imu_to_ant_offset.uncertainty_x << " "
          << localizaiton_param_.imu_to_ant_offset.uncertainty_y << " "
          << localizaiton_param_.imu_to_ant_offset.uncertainty_z;
  }
}

void MSFLocalization::PublishPoseBroadcastTF(
    const LocalizationEstimate &localization) {
  // broadcast tf message
  geometry_msgs::TransformStamped tf2_msg;
  tf2_msg.header.stamp = ros::Time(localization.measurement_time());
  tf2_msg.header.frame_id = FLAGS_broadcast_tf2_frame_id;
  tf2_msg.child_frame_id = FLAGS_broadcast_tf2_child_frame_id;

  tf2_msg.transform.translation.x = localization.pose().position().x();
  tf2_msg.transform.translation.y = localization.pose().position().y();
  tf2_msg.transform.translation.z = localization.pose().position().z();

  tf2_msg.transform.rotation.x = localization.pose().orientation().qx();
  tf2_msg.transform.rotation.y = localization.pose().orientation().qy();
  tf2_msg.transform.rotation.z = localization.pose().orientation().qz();
  tf2_msg.transform.rotation.w = localization.pose().orientation().qw();

  tf2_broadcaster_->sendTransform(tf2_msg);
  return;
}

void MSFLocalization::OnPointCloud(const sensor_msgs::PointCloud2 &message) {
  ++pcd_msg_index_;
  if (pcd_msg_index_ % FLAGS_point_cloud_step != 0) {
    return;
  }

  localization_integ_.PcdProcess(message);

  if (FLAGS_lidar_debug_log_flag) {
    std::list<LocalizationResult> lidar_localization_list;
    localization_integ_.GetLidarLocalizationList(lidar_localization_list);

    auto itr = lidar_localization_list.begin();
    auto itr_end = lidar_localization_list.end();
    for (; itr != itr_end; ++itr) {
      if (itr->state() == LocalizationMeasureState::OK) {
        // publish lidar message to debug
        AdapterManager::PublishLocalizationMsfLidar(itr->localization());
      }
    }
  }

  return;
}

void MSFLocalization::OnRawImu(const drivers::gnss::Imu &imu_msg) {
  if (FLAGS_imu_coord_rfu) {
    localization_integ_.RawImuProcessRfu(imu_msg);
  } else {
    localization_integ_.RawImuProcessFlu(imu_msg);
  }

  std::list<LocalizationResult> integ_localization_list;
  localization_integ_.GetIntegLocalizationList(integ_localization_list);

  auto itr = integ_localization_list.begin();
  auto itr_end = integ_localization_list.end();
  for (; itr != itr_end; ++itr) {
    if (itr->state() == LocalizationMeasureState::OK) {
      PublishPoseBroadcastTF(itr->localization());
      AdapterManager::PublishLocalization(itr->localization());
    }
  }

  if (integ_localization_list.size()) {
    localization_state_ = integ_localization_list.back().state();
  }

  return;
}  // namespace localization

void MSFLocalization::OnGnssBestPose(const GnssBestPose &bestgnsspos_msg) {
  if (localization_state_ == LocalizationMeasureState::OK &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.GnssBestPoseProcess(bestgnsspos_msg);

  if (FLAGS_gnss_debug_log_flag) {
    std::list<LocalizationResult> gnss_localization_list;
    localization_integ_.GetGnssLocalizationList(gnss_localization_list);

    auto itr = gnss_localization_list.begin();
    auto itr_end = gnss_localization_list.end();
    for (; itr != itr_end; ++itr) {
      if (itr->state() == LocalizationMeasureState::OK) {
        AdapterManager::PublishLocalizationMsfGnss(itr->localization());
      }
    }
  }

  return;
}

void MSFLocalization::OnGnssRtkObs(const EpochObservation &raw_obs_msg) {
  if (localization_state_ == LocalizationMeasureState::OK &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.RawObservationProcess(raw_obs_msg);

  if (FLAGS_gnss_debug_log_flag) {
    std::list<LocalizationResult> gnss_localization_list;
    localization_integ_.GetGnssLocalizationList(gnss_localization_list);

    auto itr = gnss_localization_list.begin();
    auto itr_end = gnss_localization_list.end();
    for (; itr != itr_end; ++itr) {
      if (itr->state() == LocalizationMeasureState::OK) {
        AdapterManager::PublishLocalizationMsfGnss(itr->localization());
      }
    }
  }

  return;
}

void MSFLocalization::OnGnssRtkEph(const GnssEphemeris &gnss_orbit_msg) {
  if (localization_state_ == LocalizationMeasureState::OK &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.RawEphemerisProcess(gnss_orbit_msg);
  return;
}

}  // namespace localization
}  // namespace apollo
