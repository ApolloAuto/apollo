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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
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
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

Status MSFLocalization::Start() {
  AdapterManager::Init(FLAGS_msf_adapter_config_file);

  Init();

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &MSFLocalization::OnTimer, this);
  common::monitor::MonitorBuffer buffer(&monitor_);

  // GPS
  if (!AdapterManager::GetGps()) {
    buffer.ERROR() << "GPS input not initialized. Check file "
                   << FLAGS_msf_adapter_config_file;
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no GPS adapter");
  }
  AdapterManager::AddGpsCallback(&MSFLocalization::OnGps, this);

  // Imu
  if (!AdapterManager::GetImu()) {
    buffer.ERROR("IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no IMU adapter");
  }
  AdapterManager::AddImuCallback(&MSFLocalization::OnImu, this);

  // Point Cloud
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback(&MSFLocalization::OnPointCloud, this);

  if (FLAGS_gnss_mode == 1) {
    // Gnss Rtk Obs
    CHECK(AdapterManager::GetGnssRtkObs()) << "GnssRtkObs is not initialized.";
    AdapterManager::AddGnssRtkObsCallback(&MSFLocalization::OnGnssRtkObs, this);

    // Gnss Rtk Eph
    CHECK(AdapterManager::GetGnssRtkEph()) << "GnssRtkEph is not initialized.";
    AdapterManager::AddGnssRtkEphCallback(&MSFLocalization::OnGnssRtkEph, this);
  } else {
    // Gnss Best Pose
    CHECK(AdapterManager::GetGnssBestPose())
        << "GnssBestPose is not initialized.";
    AdapterManager::AddGnssBestPoseCallback(&MSFLocalization::OnGnssBestPose,
                                            this);
  }

  // // Integ Measure Gnss
  // CHECK(AdapterManager::GetIntegMeasureGnss()) << "IntegMeasureGnss is not
  // initialized.";
  // AdapterManager::AddIntegMeasureGnssCallback(&MSFLocalization::OnIntegMeasureGnss,
  // this);

  // // Integ Measure Lidar
  // CHECK(AdapterManager::GetIntegMeasureLidar()) << "IntegMeasureLidar is not
  // initialized.";
  // AdapterManager::AddIntegMeasureLidarCallback(&MSFLocalization::OnIntegMeasureLidar,
  // this);

  // Integ Sins Pva
  CHECK(AdapterManager::GetIntegSinsPva())
      << "IntegSinsPva is not initialized.";
  AdapterManager::AddIntegSinsPvaCallback(&MSFLocalization::OnSinsPva, this);

  return Status::OK();
}

Status MSFLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}

void MSFLocalization::Init() {
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
  localizaiton_param_.extrinsic_imu_gnss_file =
      FLAGS_extrinsic_imu_gnss_filename;

  // lidar module
  localizaiton_param_.map_path = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  localizaiton_param_.lidar_extrinsic_file =
      common::util::TranslatePath(FLAGS_lidar_extrinsic_file);
  localizaiton_param_.lidar_height_file =
      common::util::TranslatePath(FLAGS_lidar_height_file);
  localizaiton_param_.lidar_debug_log_flag = FLAGS_lidar_debug_log_flag;
  localizaiton_param_.localization_mode = FLAGS_lidar_localization_mode;
  localizaiton_param_.map_coverage_theshold = FLAGS_lidar_map_coverage_theshold;
  localizaiton_param_.imu_lidar_max_delay_time = FLAGS_lidar_imu_max_delay_time;

  // common
  localizaiton_param_.utm_zone_id = FLAGS_local_utm_zone_id;
  localizaiton_param_.imu_rate = FLAGS_imu_rate;

  localization_integ_.Init(localizaiton_param_);

  // lidar_param_.Init();
  // lidar_process_.Init(lidar_param_);

  // republish_param_.Init();
  // republish_process_.Init(republish_param_);

  // integ_param_.Init();
  // integ_process_.Init(integ_param_);

  // if (FLAGS_gnss_mode == int(GnssMode::SELF)) {
  //   gnss_param_.Init();
  //   gnss_process_.Init(gnss_param_);
  // }
}

void MSFLocalization::OnTimer(const ros::TimerEvent &event) {}

void MSFLocalization::OnPointCloud(const sensor_msgs::PointCloud2 &message) {
  localization_integ_.PcdProcess(message);

  LocalizaitonMeasureState state;
  IntegMeasure lidar_measure;
  localization_integ_.GetLidarMeasure(state, lidar_measure);

  if (state == LocalizaitonMeasureState::OK) {
    // publish lidar message to debug
    AdapterManager::PublishIntegMeasureLidar(lidar_measure);
  }

  // // lidar -> republish -> integ
  // lidar_process_.PcdProcess(message);

  // // Eigen::Affine3D location;
  // // Eigen::Matrix3d covariance;
  // // int state = -1;
  // // lidar_process_.GetResult(state, location, covariance);

  // int state = 0;
  // LocalizationEstimate lidar_localization;
  // state = lidar_process_.GetResult(lidar_localization);

  // if (state == 2) {
  //   // TODO republish refactoring
  //   IntegMeasure lidar_measure;
  //   republish_process_.LidarLocalProcess(lidar_localization, lidar_measure);
  //   integ_process_.MeasureDataProcess(lidar_measure);

  //   // publish lidar message to debug
  //   AdapterManager::PublishIntegMeasureLidar(lidar_measure);
  // }
  return;
}

void MSFLocalization::OnImu(const localization::Imu &imu_msg) {
  localization_integ_.RawImuProcess(imu_msg);

  LocalizaitonMeasureState state;
  IntegSinsPva sins_pva;
  LocalizationEstimate integ_localization;
  localization_integ_.GetIntegMeasure(state, sins_pva, integ_localization);

  if (state != LocalizaitonMeasureState::NOT_VALID) {
    // publish sins_pva for debug
    AdapterManager::PublishIntegSinsPva(sins_pva);
  }

  if (state == LocalizaitonMeasureState::OK) {
    AdapterManager::PublishLocalization(integ_localization);
  }

  // imu -> lidar
  // imu -> integ -> republish -> lidar -> publish

  // lidar_process_.RawImuProcess(imu_msg);

  // integ_process_.RawImuProcess(imu_msg);

  // IntegState state;
  // IntegSinsPva sins_pva;
  // LocalizationEstimate localization;
  // integ_process_.GetResult(state, sins_pva, localization);

  // if (state != IntegState::NOT_INIT) {
  //   // update republish
  //   republish_process_.IntegPvaProcess(sins_pva);

  //   // update lidar
  //   lidar_process_.IntegPvaProcess(sins_pva);

  //   if (FLAGS_gnss_mode == int(GnssMode::SELF)) {
  //     // update gnss
  //     gnss_process_.IntegSinsPvaProcess(sins_pva);
  //   }

  //   // publish sins_pva for debug
  //   AdapterManager::PublishIntegSinsPva(sins_pva);
  // }

  // if (state == IntegState::OK) {
  //   //localization
  //   AdapterManager::PublishLocalization(localization);
  // }
  return;
}

void MSFLocalization::OnGnssBestPose(const GnssBestPose &bestgnsspos_msg) {
  localization_integ_.GnssBestPoseProcess(bestgnsspos_msg);

  // // TODO use GnssInfoType to on/of callback
  // IntegMeasure measure;
  // republish_process_.NovatelBestgnssposProcess(
  //     bestgnsspos_msg, measure);
  // integ_process_.MeasureDataProcess(measure);

  return;
}

void MSFLocalization::OnGnssRtkObs(const EpochObservation &raw_obs_msg) {
  localization_integ_.RawObservationProcess(raw_obs_msg);

  LocalizaitonMeasureState state;
  IntegMeasure measure;
  localization_integ_.GetGnssMeasure(state, measure);

  AdapterManager::PublishIntegMeasureGnss(measure);

  // gnss_process_.RawObservationProcess(raw_obs_msg);

  // IntegMeasure gnss_measure;
  // gnss_process_.GetResult(gnss_measure);

  // IntegMeasure measure;
  // republish_process_.GnssLocalProcess(gnss_measure, measure);
  // integ_process_.MeasureDataProcess(measure);

  // // pushlish GNSS IntegMeasure for debug
  // AdapterManager::PublishIntegMeasureGnss(measure);

  return;
}

void MSFLocalization::OnGnssRtkEph(const GnssEphemeris &gnss_orbit_msg) {
  localization_integ_.RawEphemerisProcess(gnss_orbit_msg);

  // gnss_process_.RawEphemerisProcess(gnss_orbit_msg);
  return;
}

void MSFLocalization::OnGps(const localization::Gps &gps_msg) {}

void MSFLocalization::OnMeasure(const localization::IntegMeasure &measure_msg) {
}

void MSFLocalization::OnSinsPva(
    const localization::IntegSinsPva &sins_pva_msg) {}

}  // namespace localization
}  // namespace apollo
