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

#include "modules/localization/common/localization_gflags.h"

DEFINE_string(localization_module_name, "localization",
              "localization module name");

DEFINE_double(localization_publish_freq, 100,
              "localization publishing frequency.");

DEFINE_string(rtk_adapter_config_file,
              "modules/localization/conf/rtk_adapter.conf",
              "rtk adapter configuration");

DEFINE_string(camera_adapter_config_file,
              "modules/localization/conf/camera_adapter.conf",
              "camera based localization adapter configuration");

DEFINE_string(camera_parameter_config_file,
              "modules/localization/conf/camera_parameter.conf",
              "camera parameter config in camera based localiztion");

DEFINE_string(localization_config_file,
              "modules/localization/conf/localization_config.pb.txt",
              "localization config file");

// features
DEFINE_bool(enable_gps_imu_interprolate, true, "enable GPU/IMU interprolate");
DEFINE_bool(enable_map_reference_unify, true,
            "enable IMU data convert to map reference");
DEFINE_bool(enable_watchdog, true, "enable watchdog");

DEFINE_double(gps_time_delay_tolerance, 1.0,
              "gps message time delay tolerance (sec)");

DEFINE_double(imu_time_delay_tolerance, 1.0,
              "imu message time delay tolerance (sec)");

DEFINE_double(camera_time_delay_tolerance, 1.0,
              "camera imu message time delay tolerance (sec)");

DEFINE_double(gps_imu_timestamp_sec_diff_tolerance, 20e-3,
              "gps/imu timestamp diff tolerance (sec)");

DEFINE_double(timestamp_sec_tolerance, 10e-7, "timestamp second tolerance");
// map offset
DEFINE_double(map_offset_x, 0.0, "map_offsite: x");
DEFINE_double(map_offset_y, 0.0, "map_offsite: y");
DEFINE_double(map_offset_z, 0.0, "map_offsite: z");

DEFINE_int32(monitor_level, 2,
             "minimum log level(INFO=0; WARN=1;"
             "ERROR=2;FATAL=3) for monitor msg");

DEFINE_int32(report_threshold_err_num, 10, "report threshold error num");
DEFINE_double(report_gps_imu_time_diff_threshold, 0.02,
              "report threshold of timestamp diff between gps and imu(sec)");

DEFINE_bool(enable_gps_timestamp, false,
            "True to set gps timestamp as localization header timestamp");

DEFINE_bool(enable_camera_timestamp, false,
            "True to set camera timestamp as localization header timestamp");
