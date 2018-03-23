#include "localization_integ.h"
#include "localization_integ_impl.h"
#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/localization/msf/common/util/time_conversion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace msf {

LocalizationInteg::LocalizationInteg() {
  localization_integ_impl_ = new LocalizationIntegImpl;
}

LocalizationInteg::~LocalizationInteg() {
  delete localization_integ_impl_;
}

LocalizationState LocalizationInteg::Init(const LocalizationIntegParam& params) {
  return localization_integ_impl_->Init(params);
}

void LocalizationInteg::PcdProcess(const sensor_msgs::PointCloud2& message) {
  LidarFrame lidar_frame;
  TransferPointCloud(message, &lidar_frame);
  localization_integ_impl_->PcdProcess(lidar_frame);
  return;
}

void LocalizationInteg::RawImuProcessFlu(const drivers::gnss::Imu& imu_msg) {
  ImuData imu;
  TransferImuFlu(imu_msg, &imu);
  localization_integ_impl_->RawImuProcessRfu(imu);
  return;
}

void LocalizationInteg::RawImuProcessRfu(const drivers::gnss::Imu& imu_msg) {
  ImuData imu;
  TransferImuRfu(imu_msg, &imu);
  localization_integ_impl_->RawImuProcessRfu(imu);
  return;
}

void LocalizationInteg::RawObservationProcess(
    const drivers::gnss::EpochObservation& raw_obs_msg) {
  localization_integ_impl_->RawObservationProcess(raw_obs_msg);
  return;
}

void LocalizationInteg::RawEphemerisProcess(
    const drivers::gnss::GnssEphemeris& gnss_orbit_msg) {
  localization_integ_impl_->RawEphemerisProcess(gnss_orbit_msg);
  return;
}

void LocalizationInteg::GnssBestPoseProcess(
    const drivers::gnss::GnssBestPose& bestgnsspos_msg) {
  localization_integ_impl_->GnssBestPoseProcess(bestgnsspos_msg);
  return;
}

void LocalizationInteg::GetLastestLidarLocalization(LocalizationMeasureState& state,
                                        LocalizationEstimate& lidar_localization) {
  localization_integ_impl_->GetLastestLidarLocalization(state, lidar_localization);
  return;
}

void LocalizationInteg::GetLastestIntegLocalization(
    LocalizationMeasureState& state,
    LocalizationEstimate& integ_localization) {
  localization_integ_impl_->GetLastestIntegLocalization(state, integ_localization);
  return;
}

void LocalizationInteg::GetLastestGnssLocalization(LocalizationMeasureState& state,
                                       LocalizationEstimate& gnss_localization) {
  localization_integ_impl_->GetLastestGnssLocalization(state, gnss_localization);
  return;
}

void LocalizationInteg::GetLidarLocalizationList(std::list<LocalizationResult>& results) {
  localization_integ_impl_->GetLidarLocalizationList(results);
  return;
}

void LocalizationInteg::GetIntegLocalizationList(std::list<LocalizationResult>& results) {
  localization_integ_impl_->GetIntegLocalizationList(results);
  return;
}

void LocalizationInteg::GetGnssLocalizationList(std::list<LocalizationResult>& results) {
  localization_integ_impl_->GetGnssLocalizationList(results);
  return;
}

void LocalizationInteg::TransferImuRfu(const drivers::gnss::Imu &imu_msg, 
                                       ImuData *imu_rfu) {
  double measurement_time = util::GpsToUnixSeconds(imu_msg.measurement_time());
  imu_rfu->measurement_time = measurement_time;
  imu_rfu->fb[0] = imu_msg.linear_acceleration().x();
  imu_rfu->fb[1] = imu_msg.linear_acceleration().y();
  imu_rfu->fb[2] = imu_msg.linear_acceleration().z();

  imu_rfu->wibb[0] = imu_msg.angular_velocity().x();  // * imu_rate_;
  imu_rfu->wibb[1] = imu_msg.angular_velocity().y();  // * imu_rate_;
  imu_rfu->wibb[2] = imu_msg.angular_velocity().z();  // * imu_rate_;
  return;
}
  
void LocalizationInteg::TransferImuFlu(const drivers::gnss::Imu &imu_msg, 
                                       ImuData *imu_flu) {
  double measurement_time = util::GpsToUnixSeconds(imu_msg.measurement_time());
  imu_flu->measurement_time = measurement_time;
  imu_flu->fb[0] = -imu_msg.linear_acceleration().y();
  imu_flu->fb[1] = imu_msg.linear_acceleration().x();
  imu_flu->fb[2] = imu_msg.linear_acceleration().z();

  imu_flu->wibb[0] = -imu_msg.angular_velocity().y();  // * imu_rate_;
  imu_flu->wibb[1] = imu_msg.angular_velocity().x();   // * imu_rate_;
  imu_flu->wibb[2] = imu_msg.angular_velocity().z();   // * imu_rate_;
  return;
}
  
void LocalizationInteg::TransferPointCloud(const sensor_msgs::PointCloud2 &lidar_data, 
                          LidarFrame *lidar_frame) {
  int total = lidar_data.width * lidar_data.height;
  int width = lidar_data.width;
  int height = lidar_data.height;
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int t_offset = -1;
  int i_offset = -1;
  int8_t x_datatype;
  int8_t y_datatype;
  int8_t z_datatype;
  int x_count = 0;
  int y_count = 0;
  int z_count = 0;
  for (std::size_t i = 0; i < lidar_data.fields.size(); ++i) {
    const sensor_msgs::PointField& f = lidar_data.fields[i];
    if (f.name == "x") {
      x_offset = f.offset;
      x_datatype = f.datatype;
      x_count = f.count;
    } else if (f.name == "y") {
      y_offset = f.offset;
      y_datatype = f.datatype;
      y_count = f.count;
    } else if (f.name == "z") {
      z_offset = f.offset;
      z_datatype = f.datatype;
      z_count = f.count;
    } else if (f.name == "timestamp") {
      t_offset = f.offset;
    } else if (f.name == "intensity") {
      i_offset = f.offset;
    }
  }
  assert(x_offset != -1 && y_offset != -1 && z_offset != -1 && t_offset != -1);
  assert(x_datatype == y_datatype && y_datatype == z_datatype);
  assert(x_datatype == 7 || x_datatype == 8);
  assert(x_count == 1 && y_count == 1 && z_count == 1);

  int num_cached = 0;
  std::map<double, TransformD> transform_cache;
  if (lidar_data.height > 1 && lidar_data.width > 1) {
    if (x_datatype == sensor_msgs::PointField::FLOAT32) {
      for (int i = 0; i < lidar_data.height; ++i) {
        for (int j = 0; j < lidar_data.width; ++j) {
          int index = i * lidar_data.width + j;
          Vector3D pt3d;
          int offset = index * lidar_data.point_step;
          pt3d[0] = static_cast<const double>(*reinterpret_cast<const float*>(
              &lidar_data.data[offset + x_offset]));
          pt3d[1] = static_cast<const double>(*reinterpret_cast<const float*>(
              &lidar_data.data[offset + y_offset]));
          pt3d[2] = static_cast<const double>(*reinterpret_cast<const float*>(
              &lidar_data.data[offset + z_offset]));
          if (!std::isnan(pt3d[0])) {
            unsigned char intensity = *reinterpret_cast<const unsigned char*>(
                &lidar_data.data[offset + i_offset]);
            lidar_frame->pt_xs.push_back(pt3d[0]);
            lidar_frame->pt_ys.push_back(pt3d[1]);
            lidar_frame->pt_zs.push_back(pt3d[2]);
            lidar_frame->intensities.push_back(intensity);
          }
        }
      }
    } else if (x_datatype == sensor_msgs::PointField::FLOAT64) {
      for (int i = 0; i < lidar_data.height; ++i) {
        for (int j = 0; j < lidar_data.width; ++j) {
          int index = i * lidar_data.width + j;
          Vector3D pt3d;
          int offset = index * lidar_data.point_step;
          pt3d[0] = *reinterpret_cast<const double*>(
              &lidar_data.data[offset + x_offset]);
          pt3d[1] = *reinterpret_cast<const double*>(
              &lidar_data.data[offset + y_offset]);
          pt3d[2] = *reinterpret_cast<const double*>(
              &lidar_data.data[offset + z_offset]);
          if (!std::isnan(pt3d[0])) {
            unsigned char intensity = *reinterpret_cast<const unsigned char*>(
                &lidar_data.data[offset + i_offset]);
            lidar_frame->pt_xs.push_back(pt3d[0]);
            lidar_frame->pt_ys.push_back(pt3d[1]);
            lidar_frame->pt_zs.push_back(pt3d[2]);
            lidar_frame->intensities.push_back(intensity);
          }
        }
      }
    } else {
      LOG(ERROR) << "The point cloud data type is not right!";
    }
  } else {
    LOG(INFO) << "Receiving un-origanized-point-cloud, width "
              << lidar_data.width << " height " << lidar_data.height;
    if (x_datatype == sensor_msgs::PointField::FLOAT32) {
      for (int i = 0; i < lidar_data.height; ++i) {
        for (int j = 0; j < lidar_data.width; ++j) {
          int index = i * lidar_data.width + j;
          Vector3D pt3d;
          int offset = index * lidar_data.point_step;
          pt3d[0] = static_cast<const double>(*reinterpret_cast<const float*>(
              &lidar_data.data[offset + x_offset]));
          pt3d[1] = static_cast<const double>(*reinterpret_cast<const float*>(
              &lidar_data.data[offset + y_offset]));
          pt3d[2] = static_cast<const double>(*reinterpret_cast<const float*>(
              &lidar_data.data[offset + z_offset]));
          if (!std::isnan(pt3d[0])) {
            unsigned char intensity = *reinterpret_cast<const unsigned char*>(
                &lidar_data.data[offset + i_offset]);
            lidar_frame->pt_xs.push_back(pt3d[0]);
            lidar_frame->pt_ys.push_back(pt3d[1]);
            lidar_frame->pt_zs.push_back(pt3d[2]);
            lidar_frame->intensities.push_back(intensity);
          }
        }
      }
    } else if (x_datatype == sensor_msgs::PointField::FLOAT64) {
      for (int i = 0; i < lidar_data.height; ++i) {
        for (int j = 0; j < lidar_data.width; ++j) {
          int index = i * lidar_data.width + j;
          Vector3D pt3d;
          int offset = index * lidar_data.point_step;
          pt3d[0] = *reinterpret_cast<const double*>(
              &lidar_data.data[offset + x_offset]);
          pt3d[1] = *reinterpret_cast<const double*>(
              &lidar_data.data[offset + y_offset]);
          pt3d[2] = *reinterpret_cast<const double*>(
              &lidar_data.data[offset + z_offset]);
          if (!std::isnan(pt3d[0])) {
            unsigned char intensity = *reinterpret_cast<const unsigned char*>(
                &lidar_data.data[offset + i_offset]);
            lidar_frame->pt_xs.push_back(pt3d[0]);
            lidar_frame->pt_ys.push_back(pt3d[1]);
            lidar_frame->pt_zs.push_back(pt3d[2]);
            lidar_frame->intensities.push_back(intensity);
          }
        }
      }
    } else {
      LOG(ERROR) << "The point cloud data type is not right!";
    }
  }
  lidar_frame->measurement_time = lidar_data.header.stamp.toSec();
  if (FLAGS_lidar_debug_log_flag) {
    LOG(INFO) << std::setprecision(16)
              << "Localization10Hz Debug Log: lidar msg. "
              << "[time:" << lidar_frame->measurement_time << "]"
              << "[height:" << lidar_data.height << "]"
              << "[width:" << lidar_data.width << "]"
              << "[point_step:" << lidar_data.point_step << "]"
              << "[data_type:" << x_datatype << "]"
              << "[point_cnt:"
              << static_cast<unsigned int>(lidar_frame->pt_xs.size()) << "]"
              << "[intensity_cnt:"
              << static_cast<unsigned int>(lidar_frame->intensities.size())
              << "]";
    // if (lidar_frame->pt3ds.size() > 0) {
    //   LOG(INFO) << "Localization10Hz Debug Log: lidar msg first point info. "
    //             << "[x:" << lidar_frame->pt3ds[0][0] << "]"
    //             << "[y:" << lidar_frame->pt3ds[0][1] << "]"
    //             << "[z:" << lidar_frame->pt3ds[0][2] << "]";
    // }
  }
  return;
}
  

}  // namespace msf
}  // namespace localization
}  // namespace apollo
