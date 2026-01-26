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

#include "modules/dreamview/backend/common/dreamview_gflags.h"

DEFINE_string(dreamview_module_name, "dreamview", "dreamview module name");

DEFINE_bool(dreamview_profiling_mode, false, "Run dreamview in profiling mode");

DEFINE_int32(
    dreamview_profiling_duration, -1,
    "Dreamview profiling duration in ms. Negative value will not restrict the "
    "profiling time");

DEFINE_string(static_file_dir, "/apollo/modules/dreamview_plus/frontend/dist",
              "The path to the dreamview distribution directory. The default "
              "value points to built-in version from the Apollo project.");

DEFINE_string(
    server_ports, "8888",
    "Comma-separated list of ports to listen on. If the port is SSL, "
    "a letter s must be appended, for example, 80,443s will open "
    "port 80 and port 443.Dreamview always use 8899 and Dreamview Plus"
    " always use 8888.");

DEFINE_bool(routing_from_file, false,
            "Whether Dreamview reads initial routing response from file.");

DEFINE_string(routing_response_file,
              "modules/map/data/demo/garage_routing.pb.txt",
              "File path of the routing response that SimControl will read the "
              "start point from. If this is absent, SimControl will directly "
              "take the RoutingResponse from external to determine the start "
              "point.");

DEFINE_string(websocket_timeout_ms, "36000000",
              "Time span that CivetServer keeps the websocket connection alive "
              "without dropping it.");

DEFINE_string(ssl_certificate, "",
              "Path to the SSL certificate file. This option is only required "
              "when at least one of the listening_ports is SSL. The file must "
              "be in PEM format, and it must have both, private key and "
              "certificate");

DEFINE_double(sim_map_radius, 200.0,
              "The radius within which Dreamview will find all the map "
              "elements around the car.");

DEFINE_bool(enable_update_size_check, true,
            "True to check if the update byte number is less than threshold");

DEFINE_uint32(max_update_size, 1000000,
              "Number of max update bytes allowed to push to dreamview FE");

DEFINE_bool(sim_world_with_routing_path, false,
            "Whether the routing_path is included in sim_world proto.");

DEFINE_string(
    request_timeout_ms, "2000",
    "Timeout for network read and network write operations, in milliseconds.");

DEFINE_double(voxel_filter_size, 0.3, "VoxelGrid pointcloud filter leaf size");

DEFINE_double(voxel_filter_height, 0.2,
              "VoxelGrid pointcloud filter leaf height");

DEFINE_double(system_status_lifetime_seconds, 30,
              "Lifetime of a valid SystemStatus message. It's more like a "
              "replay message if the timestamp is old, where we should ignore "
              "the status change.");

DEFINE_string(lidar_height_yaml,
              "/apollo/modules/localization/msf/params/velodyne_params/"
              "velodyne64_height.yaml",
              "The yaml file for reading height of lidar w.r.t. ground.");

DEFINE_int32(monitor_msg_pending_queue_size, 10,
             "Max monitor message pending queue size");

DEFINE_string(default_data_collection_config_path,
              "/apollo/modules/dreamview_plus/conf/data_collection_table.pb.txt",
              "Data collection table config path.");

DEFINE_int32(loop_routing_end_to_start_distance_threshold, 10,
             "Loop routing distance threshold: start to end");

DEFINE_string(default_preprocess_config_path,
              "/apollo/modules/dreamview_plus/conf/preprocess_table.pb.txt",
              "Sensor calibration preprocess table config path.");

DEFINE_string(vehicle_calibration_mode, "Vehicle Calibration",
              "Name of vehicle calibration mode.");

DEFINE_string(lidar_calibration_mode, "Lidar-IMU Sensor Calibration",
              "Name of lidar_to_gnss calibration mode.");

DEFINE_string(camera_calibration_mode, "Camera-Lidar Sensor Calibration",
              "Name of camera_to_lidar calibration mode.");

DEFINE_double(parking_routing_distance_threshold, 20.0,
              "For open space planner parking situation: get the routing"
              "end point based on this threshold.");

DEFINE_string(plugin_path, "/.apollo/dreamview/plugins/", "Plugin placement");

DEFINE_string(plugin_config_file_name_suffix, "_plugin_config.pb.txt",
              "PLugin config file name!");

DEFINE_string(plugin_channel_prefix, "/apollo/dreamview/plugins/",
              "plugins must use this as channel prefix");

DEFINE_string(resource_scenario_path, "/.apollo/resources/scenario_sets/",
              "Scenario set placement");

DEFINE_string(resource_dynamic_model_path,
              "/.apollo/resources/dynamic_models/models/",
              "Dynamic Models placement");

DEFINE_string(dynamic_model_root_path, "/.apollo/resources/dynamic_models/",
              "Dynamic Model root directory");

DEFINE_string(dynamic_model_library_path,
              "/.apollo/resources/dynamic_models/library/",
              "Dynamic Model libs placement");

DEFINE_string(dynamic_model_package_library_path,
              "/opt/apollo/neo/lib/modules/dynamic_model/",
              "Dynamic Model package lib path");

DEFINE_string(apollo_package_meta_info_path_prefix,
              "/opt/apollo/neo/share/packages/",
              "apollo package meta info path prefix");

DEFINE_string(sim_obstacle_stop_command, "pkill -9 -f \"sim_obstacle\" ",
              "Sim obstacle stop command");

DEFINE_string(sim_obstacle_path,
              "/opt/apollo/neo/bin/sim_obstacle",
              "sim obstacle binary placement.");

DEFINE_string(gflag_command_arg,
              " --flagfile=/apollo/modules/common/data/global_flagfile.txt",
              "sim obstacle need use gflag!");

DEFINE_string(sim_perfect_control, "Simulation Perfect Control",
              "sim perfect control!");

DEFINE_string(resource_record_path, "/.apollo/resources/records/",
              "Records placement");

DEFINE_string(resource_rtk_record_path, "/apollo/data/log",
              "Waypoint Follow Records placement");

DEFINE_string(cyber_recorder_stop_command, "pkill -9 cyber_recorder",
              "stop play recorder");

DEFINE_string(vehicles_config_path, "/apollo/modules/calibration/data",
              "Vehicles config path.");

DEFINE_bool(
    vehicle_changed_use_copy_mode, true,
    "change vehicle use copy mode if set to true, else use symlink mode");

DEFINE_string(lane_follow_command_topic, "/apollo/external_command/lane_follow",
              "Lane follow command topic name.");

DEFINE_string(valet_parking_command_topic,
              "/apollo/external_command/valet_parking",
              "Valet parking command topic name.");

DEFINE_string(action_command_topic, "/apollo/external_command/action",
              "Action command topic name.");
DEFINE_string(data_handler_config_path, "/apollo/modules/dreamview_plus/conf/data_handler.conf",
              "Data handler config path.");

DEFINE_string(data_recorder_command_keyword, "cyber_recorder record",
              "Data recorder command keyword.");

DEFINE_string(data_record_default_name, "default_record_name",
              "Data record default name");

DEFINE_double(threshold_for_destination_check, 1.0,
              "meters, which is 100 feet.  This threshold is used to check if"
              "the vehicle reaches the destination");
DEFINE_string(dv_hmi_modes_config_path,
              "/apollo/modules/dreamview/conf/hmi_modes",
              "Dreamview HMI modes config path.");
DEFINE_string(dv_plus_hmi_modes_config_path,
              "/apollo/modules/dreamview_plus/conf/hmi_modes",
              "Dreamview Plus HMI modes config path.");
DEFINE_string(maps_data_path, "/apollo/modules/map/data", "Maps data path.");
DEFINE_string(global_components_config_path,
              "/apollo/modules/dreamview/conf/global_components_config.pb.txt",
              "Global components config path.");

DEFINE_string(
    terminal_start_cmd,
    "nohup /apollo/modules/dreamview_plus/backend/third_party_lib/cyber_shell "
    "&",
    "Terminal start cmd");

DEFINE_string(terminal_stop_cmd, "pkill -9 -f \"cyber_shell\" ",
              "Terminal start cmd");

DEFINE_string(cyber_channels_key, "apollo.dreamview.Cyber",
              "Cyber channels key");
DEFINE_string(vehicle_data_config_filename,
              "/apollo/modules/dreamview_plus/conf/vehicle_data.pb.txt",
              "Vehicle data config file.");
            
DEFINE_double(status_publish_interval, 5, "HMI Status publish interval.");

DEFINE_string(current_mode_db_key, "/apollo/hmi/status:current_mode",
              "Key to store hmi_status.current_mode in KV DB.");

DEFINE_string(default_hmi_mode, "Default",
              "Default HMI Mode when there is no cache.");

DEFINE_string(default_rtk_record_file, "/apollo/data/log/garage.csv",
              "Default rtk record file.");

DEFINE_string(default_rtk_record_path, "/apollo/data/log/",
              "Default rtk record path.");

DEFINE_bool(dv_cpu_profile, false, "enable cpu profile");

DEFINE_bool(dv_heap_profile, false, "enable heap profile");

DEFINE_double(
    monitor_timeout_threshold, 2.5,
    "Monitor the maximum tolerable time from the last message sent. If "
    "it exceeds the time, the autonomous driving will be exited.");
