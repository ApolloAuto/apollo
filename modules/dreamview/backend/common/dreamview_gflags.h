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

#include "gflags/gflags.h"

DECLARE_string(dreamview_module_name);

DECLARE_bool(dreamview_profiling_mode);

DECLARE_int32(dreamview_profiling_duration);

DECLARE_string(static_file_dir);

DECLARE_string(server_ports);

DECLARE_bool(routing_from_file);

DECLARE_string(routing_response_file);

DECLARE_string(websocket_timeout_ms);

DECLARE_string(ssl_certificate);

DECLARE_double(sim_map_radius);

DECLARE_int32(dreamview_worker_num);

DECLARE_bool(enable_update_size_check);

DECLARE_uint32(max_update_size);

DECLARE_bool(sim_world_with_routing_path);

DECLARE_string(request_timeout_ms);

DECLARE_double(voxel_filter_size);

DECLARE_double(voxel_filter_height);

DECLARE_double(system_status_lifetime_seconds);

DECLARE_string(lidar_height_yaml);

DECLARE_int32(monitor_msg_pending_queue_size);

DECLARE_string(default_data_collection_config_path);

DECLARE_int32(loop_routing_end_to_start_distance_threshold);

DECLARE_string(default_preprocess_config_path);

DECLARE_string(vehicle_calibration_mode);

DECLARE_string(lidar_calibration_mode);

DECLARE_string(camera_calibration_mode);

DECLARE_double(parking_routing_distance_threshold);

DECLARE_string(plugin_path);

DECLARE_string(plugin_config_file_name_suffix);

DECLARE_string(plugin_channel_prefix);

DECLARE_string(resource_scenario_path);

DECLARE_string(resource_dynamic_model_path);

DECLARE_string(dynamic_model_root_path);

DECLARE_string(dynamic_model_library_path);

DECLARE_string(dynamic_model_package_library_path);

DECLARE_string(apollo_package_meta_info_path_prefix);

DECLARE_string(sim_obstacle_stop_command);

DECLARE_string(sim_obstacle_path);

DECLARE_string(gflag_command_arg);

DECLARE_string(sim_perfect_control);

DECLARE_string(resource_record_path);

DECLARE_string(resource_rtk_record_path);

DECLARE_string(cyber_recorder_stop_command);

DECLARE_string(vehicles_config_path);

DECLARE_bool(vehicle_changed_use_copy_mode);

DECLARE_string(lane_follow_command_topic);

DECLARE_string(valet_parking_command_topic);

DECLARE_string(action_command_topic);
DECLARE_string(data_handler_config_path);

DECLARE_string(data_recorder_command_keyword);

DECLARE_string(data_record_default_name);

DECLARE_double(threshold_for_destination_check);

DECLARE_string(dv_hmi_modes_config_path);

DECLARE_string(dv_plus_hmi_modes_config_path);

DECLARE_string(maps_data_path);

DECLARE_string(global_components_config_path);

DECLARE_string(terminal_start_cmd);

DECLARE_string(terminal_stop_cmd);

DECLARE_string(cyber_channels_key);

DECLARE_string(vehicle_data_config_filename);

DECLARE_double(status_publish_interval);

DECLARE_string(current_mode_db_key);

DECLARE_string(default_hmi_mode);

DECLARE_string(default_rtk_record_file);

DECLARE_string(default_rtk_record_path);

DECLARE_bool(dv_cpu_profile);

DECLARE_bool(dv_heap_profile);

DECLARE_double(monitor_timeout_threshold);
