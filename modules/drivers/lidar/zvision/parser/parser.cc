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

#include "cyber/cyber.h"

#include "modules/drivers/lidar/zvision/parser/parser.h"
#include "modules/drivers/lidar/zvision/tools/tools.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      // return the timestamp(utc nanosecond) in udp packet
      uint64_t ZvisionParser::GetUdpSensorTimestamp(const unsigned char *data)
      {
        int gps_ptp_status_2_bits_ = ((data[2] & 0x30) >> 4);

        // timegm()
        if (gps_ptp_status_2_bits_ & 0x2) // PTP
        {
          uint64_t seconds = 0;
          seconds = 0;
          seconds += ((uint64_t)data[1304 - 20 + 0] << 40);
          seconds += ((uint64_t)data[1304 - 20 + 1] << 32);
          seconds += ((uint64_t)data[1304 - 20 + 2] << 24);
          seconds += ((uint64_t)data[1304 - 20 + 3] << 16);
          seconds += ((uint64_t)data[1304 - 20 + 4] << 8);
          seconds += ((uint64_t)data[1304 - 20 + 5]);

          int MillS = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
          int MicroS = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

          return (uint64_t)seconds * 1000000000 + MillS * 1000000 + MicroS * 1000;
        }
        else // GPS
        {
          struct tm tm_;
          tm_.tm_year = data[1304 - 20 + 0];
          tm_.tm_mon = data[1304 - 20 + 1];
          tm_.tm_mday = data[1304 - 20 + 2];
          tm_.tm_hour = data[1304 - 20 + 3];
          tm_.tm_min = data[1304 - 20 + 4];
          tm_.tm_sec = data[1304 - 20 + 5];
          tm_.tm_isdst = 0;

          time_t seconds = timegm(&tm_);
          int MillS = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
          int MicroS = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

          return (uint64_t)seconds * 1000000000 + MillS * 1000000 + MicroS * 1000;
        }
      }

      double ZvisionParser::GetExcitonTimestampNSSampleA(const unsigned char *data) 
      {

          uint64_t seconds = 0;
          seconds += ((uint64_t)data[1304 + 0] << 40);
          seconds += ((uint64_t)data[1304 + 1] << 32);
          seconds += ((uint64_t)data[1304 + 2] << 24);
          seconds += ((uint64_t)data[1304 + 3] << 16);
          seconds += ((uint64_t)data[1304 + 4] << 8);
          seconds += ((uint64_t)data[1304 + 5]);

          uint32_t ms = (int)(data[1304 + 6] << 8) + data[1304 + 7];
          uint32_t us = (int)(data[1304 + 8] << 8) + data[1304 + 9];

          return (double)seconds + (double)ms / 1000.0 + (double)us / 1000000.0;
      }

      double ZvisionParser::GetExcitonTimestampNSSampleB(const unsigned char *data) 
      {

          uint64_t seconds = 0;
          seconds += ((uint64_t)data[32 + 0] << 40);
          seconds += ((uint64_t)data[32 + 1] << 32);
          seconds += ((uint64_t)data[32 + 2] << 24);
          seconds += ((uint64_t)data[32 + 3] << 16);
          seconds += ((uint64_t)data[32 + 4] << 8);
          seconds += ((uint64_t)data[32 + 5]);

          uint32_t ms = (int)(data[34 + 6] << 8) + data[34 + 7];
          uint32_t us = (int)(data[34 + 8] << 8) + data[34 + 9];

          return (double)seconds + (double)ms / 1000.0 + (double)us / 1000000.0;
      }

      PointXYZIT ZvisionParser::get_nan_point(uint64_t timestamp)
      {
        PointXYZIT nan_point;
        nan_point.set_timestamp(timestamp);
        nan_point.set_x(nan);
        nan_point.set_y(nan);
        nan_point.set_z(nan);
        nan_point.set_intensity(0);
        return nan_point;
      }

      void ZvisionParser::PollCalibrationData(void)
      {
        while (!cyber::IsShutdown())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          if (is_calibration_)
            break;
          CalibrationData cal;
          int ret = 0;
          if (0 != (ret = LidarTools::GetOnlineCalibrationData(config_.ip(), cal)))
            AERROR << "Unable to get online calibration"
                   << config_.ip();
          else
          {
            LidarTools::ComputeCalibrationData(cal, cal_lut_);
            is_calibration_ = true;
            AINFO << "Get online calibration ok.";
            printf("Get online calibration ok.\n");
          }
        }
      }

      ZvisionParser::ZvisionParser(const Config &config) : config_(config)
      {
      }

      ZvisionParser::~ZvisionParser()
      {
        running_.store(false);
        if (online_calibration_thread_.joinable())
        {
          online_calibration_thread_.join();
        }
      }

      /** Set up for on-line operation. */
      void ZvisionParser::setup()
      {
        // init transform
        need_transform_ = config_.need_transform();
        if (need_transform_)
        {
          Eigen::Translation3d translation(config_.tx(), config_.ty(), config_.tz());
          Eigen::AngleAxisd roll(config_.roll() / 180.0 * M_PI, Eigen::Vector3d::UnitX());
          Eigen::AngleAxisd pitch(config_.pitch() / 180.0 * M_PI, Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd yaw(config_.yaw() / 180.0 * M_PI, Eigen::Vector3d::UnitZ());

          Eigen::Quaterniond quaterniond = yaw * pitch * roll;
          // Eigen::Matrix3d quaterniond = (yaw * pitch * roll).matrix();

          transform_ = translation * quaterniond;
        }

        // get calibration
        // if (!config_.calibration_online())
        // {
        //   CalibrationData cal;
        //   int ret = 0;
        //   if (0 == (ret = LidarTools::ReadCalibrationFile(config_.calibration_file(), cal)))
        //   {
        //     LidarTools::ComputeCalibrationData(cal, cal_lut_);
        //     is_calibration_ = true;
        //   }
        //   else
        //   {
        //     AFATAL << "Unable to open calibration file: "
        //            << config_.calibration_file();
        //   }
        // }
        // else
        // {
        //   // raw data output topic
        //   online_calibration_thread_ =
        //       std::thread(&ZvisionParser::PollCalibrationData, this);
        // }
      }

      bool ZvisionParser::CalibrationInitOk()
      {
        return is_calibration_;
      }

      void ZvisionParser::ComputeCoords(PointXYZIT *point)
      {
        /*
      // ROS_ASSERT_MSG(rotation < 36000, "rotation must between 0 and 35999");
      //assert(rotation <= 36000);
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;

      //double distance = raw_distance + corrections.dist_correction;

      // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
      // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
      double cos_rot_angle =
          cos_rot_table_[rotation] * corrections.cos_rot_correction +
          sin_rot_table_[rotation] * corrections.sin_rot_correction;
      double sin_rot_angle =
          sin_rot_table_[rotation] * corrections.cos_rot_correction -
          cos_rot_table_[rotation] * corrections.sin_rot_correction;

      // double vert_offset = corrections.vert_offset_correction;

      // Compute the distance in the xy plane (w/o accounting for rotation)
      double xy_distance = distance * corrections.cos_vert_correction;

      // Calculate temporal X, use absolute value.
      double xx = fabs(xy_distance * sin_rot_angle -
                       corrections.horiz_offset_correction * cos_rot_angle);
      // Calculate temporal Y, use absolute value
      double yy = fabs(xy_distance * cos_rot_angle +
                       corrections.horiz_offset_correction * sin_rot_angle);

      // Get 2points calibration values,Linear interpolation to get distance
      // correction for X and Y, that means distance correction use
      // different value at different distance
      double distance_corr_x = 0;
      double distance_corr_y = 0;

      if (need_two_pt_correction_ && raw_distance <= 2500) {
        distance_corr_x =
            (corrections.dist_correction - corrections.dist_correction_x) *
                (xx - 2.4) / 22.64 +
            corrections.dist_correction_x;  // 22.64 = 25.04 - 2.4
        distance_corr_y =
            (corrections.dist_correction - corrections.dist_correction_y) *
                (yy - 1.93) / 23.11 +
            corrections.dist_correction_y;  // 23.11 = 25.04 - 1.93
      } else {
        distance_corr_x = distance_corr_y = corrections.dist_correction;
      }

      double distance_x = raw_distance + distance_corr_x;

      xy_distance = distance_x * corrections.cos_vert_correction;
      // xy_distance = distance_x * cos_vert_correction - vert_offset *
      // sin_vert_correction;

      x = xy_distance * sin_rot_angle -
          corrections.horiz_offset_correction * cos_rot_angle;

      double distance_y = raw_distance + distance_corr_y;
      xy_distance = distance_y * corrections.cos_vert_correction;
      // xy_distance = distance_y * cos_vert_correction - vert_offset *
      // sin_vert_correction;
      y = xy_distance * cos_rot_angle +
          corrections.horiz_offset_correction * sin_rot_angle;

      z = distance * corrections.sin_vert_correction +
          corrections.vert_offset_correction;
      // z = distance * sin_vert_correction + vert_offset * cos_vert_correction;
    */
        /** Use standard ROS coordinate system (right-hand rule) */
        // point->set_x(static_cast<float>(y));
        // point->set_y(static_cast<float>(-x));
        // point->set_z(static_cast<float>(z));
      }

      ZvisionParser *ZvisionParserFactory::CreateParser(Config source_config)
      {
        Config config = source_config;
        if (config.model() == ML30B1)
        {
          return new Ml30Parser(config);
        }
        else if (config.model() == ML30SA1)
        {
          return new Ml30sa1Parser(config);
        }
        else if (config.model() == MLX)
        {
          return new MlxParser(config);
        }
        else if (config.model() == EZ6)
        {
          return new EZ6Parser(config);
        }
        else
        {
          AERROR << "invalid model, must be ML30|ML30SA1";
          return nullptr;
        }
      }

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
