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

#pragma once
#include <Eigen/Geometry>

namespace apollo {
namespace localization {
namespace msf {
typedef Eigen::Affine3d TransformD;
/**@brief the imu data struct including time, accelerometer and gyro in body frame. */
struct ImuData {
  ImuData(): measurement_time(0.0), fb{0.0}, wibb{0.0} {}
  double measurement_time; // unix time
  double fb[3];
  double wibb[3];
};

struct WheelspeedData {
  WheelspeedData(): time(0.0), front_right_speed(0.0),
      front_left_speed(0.0), rear_right_speed(0.0),
      rear_left_speed(0.0) {}
  double time;
  double front_right_speed;
  double front_left_speed;
  double rear_right_speed;
  double rear_left_speed;
};

/**@brief the position struct including longitude, latitude and height. */
struct Position {
  Position(): longitude(0.0), latitude(0.0), height(0.0) {}
  double longitude;
  double latitude;
  double height;
};
/**@brief the velocity struct in ENU navigation frame. */
struct Velocity {
  Velocity(): ve(0.0), vn(0.0), vu(0.0) {}
  double ve;
  double vn;
  double vu;
};
/**@brief the attitude struct including pitch, roll and yaw. */
struct Attitude {
  Attitude(): pitch(0.0), roll(0.0), yaw(0.0) {}
  double pitch;
  double roll;
  double yaw;
};

/**@brief the INS pva struct including time, sins position velocity attitude, attitude quaternion
* and the sins alignment status. */
struct InsPva {
  InsPva (): time(0.0),
      qbn{1.0, 0.0, 0.0, 0.0},
      init_and_alignment(false) {}
  double time;
  Position pos;
  Velocity vel;
  Attitude att;
  double qbn[4];
  bool init_and_alignment;
};

struct Pose {
  Pose(): x(0.0), y(0.0), z(0.0),
      qx(0.0), qy(0.0), qz(0.0), qw(1.0) {}
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

enum class SinsIntegUpdateType {
  SINS_UPDATE = 0,
  SINS_FILTER_TIME_UPDATE,
  FILTER_TIME_UPDATE,
  FILTER_MEASURE_UPDATE,
  ALL_UPDATE
};

/**@brief the measure data source. */
enum class MeasureType {
  GNSS_POS_ONLY = 0,
  GNSS_POS_VEL,
  GNSS_POS_XY,
  ENU_VEL_ONLY,
  POINT_CLOUD_POS,
  ODOMETER_VEL_ONLY,
  VEHICLE_CONSTRAINT,
  GNSS_DOUBLE_ANT_YAW,
  ZUPT
};
/**@brief the frame type. */
enum class FrameType {
  ENU = 0,      //in this frame the position give x y and z from earth center
  ECEF,     //in this frame the position give the longitude and latitude unit:rad
  UTM,      //in this frame the position give x y and z in utm frame
  ODOMETER_BODY
};

/**@brief the measure data give to the navigation kalman filter measure update. */
struct MeasureData {
  MeasureData(): time(0.0), gnss_mode(0),
      measure_type(MeasureType::GNSS_POS_ONLY),
      frame_type(FrameType::ENU),
      is_have_variance(false), variance{0.0} {}
  double time;
  Position gnss_pos;
  Velocity gnss_vel;
  Attitude gnss_att;
  int gnss_mode;                 //gnss positioning type
  MeasureType measure_type;
  FrameType frame_type;
  bool is_have_variance;
  double variance[10][10];       //the noise variance of the measurement
};

/**@brief the parameter using in sins calculate. */
struct InertialParameter {
    double wien[3];  //the rate of the earth rotation on the navigation frame
    double wenn[3];  //the rate of the carrier rotation relate to earth on the navigation frame
    double rm;
    double rn;
    double g;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo