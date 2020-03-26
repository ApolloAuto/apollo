load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "planning_proto",
    deps = [
        ":planning_proto_lib",
    ],
)

proto_library(
    name = "planning_proto_lib",
    srcs = [
        "pad_msg.proto",
        "planning.proto",
    ],
    deps = [
        ":decision_proto_lib",
        ":planning_internal_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/localization/proto:pose_proto_lib",
        "//modules/map/proto:map_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "creep_decider_config_proto",
    deps = [
        ":creep_decider_config_proto_lib",
    ],
)

proto_library(
    name = "creep_decider_config_proto_lib",
    srcs = ["creep_decider_config.proto"],
)

cc_proto_library(
    name = "decision_proto",
    deps = [
        ":decision_proto_lib",
    ],
)

proto_library(
    name = "decision_proto_lib",
    srcs = ["decision.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "dp_st_speed_config_proto",
    deps = [
        ":dp_st_speed_config_proto_lib",
    ],
)

proto_library(
    name = "dp_st_speed_config_proto_lib",
    srcs = ["dp_st_speed_config.proto"],
)

cc_proto_library(
    name = "navi_path_decider_config_proto",
    deps = [
        ":navi_path_decider_config_proto_lib",
    ],
)

proto_library(
    name = "navi_path_decider_config_proto_lib",
    srcs = ["navi_path_decider_config.proto"],
)

cc_proto_library(
    name = "navi_speed_decider_config_proto",
    deps = [
        ":navi_speed_decider_config_proto_lib",
    ],
)

proto_library(
    name = "navi_speed_decider_config_proto_lib",
    srcs = ["navi_speed_decider_config.proto"],
)

cc_proto_library(
    name = "navi_obstacle_decider_config_proto",
    deps = [
        ":navi_obstacle_decider_config_proto_lib",
    ],
)

proto_library(
    name = "navi_obstacle_decider_config_proto_lib",
    srcs = ["navi_obstacle_decider_config.proto"],
)

cc_proto_library(
    name = "planning_config_proto",
    deps = [
        ":planning_config_proto_lib",
    ],
)

proto_library(
    name = "planning_config_proto_lib",
    srcs = ["planning_config.proto"],
    deps = [
        ":creep_decider_config_proto_lib",
        ":dp_st_speed_config_proto_lib",
        ":lane_change_decider_config_proto_lib",
        ":navi_obstacle_decider_config_proto_lib",
        ":navi_path_decider_config_proto_lib",
        ":navi_speed_decider_config_proto_lib",
        ":open_space_fallback_decider_config_proto_lib",
        ":open_space_pre_stop_decider_config_proto_lib",
        ":open_space_roi_decider_config_proto_lib",
        ":open_space_trajectory_partition_config_proto_lib",
        ":open_space_trajectory_provider_config_proto_lib",
        ":path_assessment_decider_config_proto_lib",
        ":path_bounds_decider_config_proto_lib",
        ":path_decider_config_proto_lib",
        ":path_lane_borrow_decider_config_proto_lib",
        ":path_reuse_decider_config_proto_lib",
        ":piecewise_jerk_nonlinear_speed_config_proto_lib",
        ":piecewise_jerk_path_config_proto_lib",
        ":piecewise_jerk_speed_config_proto_lib",
        ":planner_open_space_config_proto_lib",
        ":reference_line_smoother_config_proto_lib",
        ":rule_based_stop_decider_config_proto_lib",
        ":speed_bounds_decider_config_proto_lib",
        ":st_bounds_decider_config_proto_lib",
    ],
)

proto_library(
    name = "planning_internal_proto_lib",
    srcs = ["planning_internal.proto"],
    deps = [
        ":decision_proto_lib",
        ":planning_config_proto_lib",
        ":sl_boundary_proto_lib",
        ":st_drivable_boundary_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/dreamview/proto:chart_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/map/relative_map/proto:navigation_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "planning_stats_proto",
    deps = [
        ":planning_stats_proto_lib",
    ],
)

proto_library(
    name = "planning_stats_proto_lib",
    srcs = ["planning_stats.proto"],
)

cc_proto_library(
    name = "reference_line_smoother_config_proto",
    deps = [
        ":reference_line_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "reference_line_smoother_config_proto_lib",
    srcs = ["reference_line_smoother_config.proto"],
    deps = [
        ":cos_theta_smoother_config_proto_lib",
        ":fem_pos_deviation_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "sl_boundary_proto_lib",
    srcs = ["sl_boundary.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

proto_library(
    name = "st_drivable_boundary_proto_lib",
    srcs = ["st_drivable_boundary.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

cc_proto_library(
    name = "traffic_rule_config_proto",
    deps = [
        ":traffic_rule_config_proto_lib",
    ],
)

proto_library(
    name = "traffic_rule_config_proto_lib",
    srcs = ["traffic_rule_config.proto"],
)

cc_proto_library(
    name = "piecewise_jerk_path_config_proto",
    deps = [
        ":piecewise_jerk_path_config_proto_lib",
    ],
)

proto_library(
    name = "piecewise_jerk_path_config_proto_lib",
    srcs = ["piecewise_jerk_path_config.proto"],
)

cc_proto_library(
    name = "piecewise_jerk_speed_config_proto",
    deps = [
        ":piecewise_jerk_speed_config_proto_lib",
    ],
)

proto_library(
    name = "piecewise_jerk_speed_config_proto_lib",
    srcs = ["piecewise_jerk_speed_config.proto"],
)

cc_proto_library(
    name = "piecewise_jerk_nonlinear_speed_config_proto",
    deps = [
        ":piecewise_jerk_nonlinear_speed_config_proto_lib",
    ],
)

proto_library(
    name = "piecewise_jerk_nonlinear_speed_config_proto_lib",
    srcs = ["piecewise_jerk_nonlinear_speed_config.proto"],
)

cc_proto_library(
    name = "path_bounds_decider_config_proto",
    deps = [
        ":path_bounds_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_bounds_decider_config_proto_lib",
    srcs = ["path_bounds_decider_config.proto"],
)

cc_proto_library(
    name = "path_decider_config_proto",
    deps = [
        ":path_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_decider_config_proto_lib",
    srcs = ["path_decider_config.proto"],
)

cc_proto_library(
    name = "path_assessment_decider_config_proto",
    deps = [
        ":path_assessment_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_assessment_decider_config_proto_lib",
    srcs = ["path_assessment_decider_config.proto"],
)

cc_proto_library(
    name = "path_lane_borrow_decider_config_proto",
    deps = [
        ":path_lane_borrow_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_lane_borrow_decider_config_proto_lib",
    srcs = ["path_lane_borrow_decider_config.proto"],
)

cc_proto_library(
    name = "path_reuse_decider_config_proto",
    deps = [
        ":path_reuse_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_reuse_decider_config_proto_lib",
    srcs = ["path_reuse_decider_config.proto"],
)

cc_proto_library(
    name = "lane_change_decider_config_proto",
    deps = [
        ":lane_change_decider_config_proto_lib",
    ],
)

proto_library(
    name = "lane_change_decider_config_proto_lib",
    srcs = ["lane_change_decider_config.proto"],
)

cc_proto_library(
    name = "spiral_curve_config_proto",
    deps = [
        ":spiral_curve_config_proto_lib",
    ],
)

proto_library(
    name = "spiral_curve_config_proto_lib",
    srcs = ["spiral_curve_config.proto"],
)

cc_proto_library(
    name = "planning_status_proto",
    deps = [
        ":planning_status_proto_lib",
    ],
)

proto_library(
    name = "planning_status_proto_lib",
    srcs = ["planning_status.proto"],
    deps = [
        ":planning_config_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "lattice_sampling_config_proto",
    deps = [
        ":lattice_sampling_config_proto_lib",
    ],
)

proto_library(
    name = "lattice_sampling_config_proto_lib",
    srcs = ["lattice_sampling_config.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

cc_proto_library(
    name = "lattice_structure_proto",
    deps = [
        ":lattice_structure_proto_lib",
    ],
)

proto_library(
    name = "lattice_structure_proto_lib",
    srcs = ["lattice_structure.proto"],
    deps = [
        ":lattice_sampling_config_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

proto_library(
    name = "planner_open_space_config_proto_lib",
    srcs = ["planner_open_space_config.proto"],
    deps = [
        ":fem_pos_deviation_smoother_config_proto_lib",
        ":piecewise_jerk_speed_config_proto_lib",
    ],
)

cc_proto_library(
    name = "planner_open_space_config_proto",
    deps = [
        ":planner_open_space_config_proto_lib",
    ],
)

cc_proto_library(
    name = "auto_tuning_raw_feature_proto",
    deps = [
        ":auto_tuning_raw_feature_lib",
    ],
)

proto_library(
    name = "auto_tuning_raw_feature_lib",
    srcs = ["auto_tuning_raw_feature.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

cc_proto_library(
    name = "auto_tuning_model_input_proto",
    deps = [
        ":auto_tuning_model_input_lib",
    ],
)

proto_library(
    name = "auto_tuning_model_input_lib",
    srcs = ["auto_tuning_model_input.proto"],
)

cc_proto_library(
    name = "qp_problem_proto",
    deps = [
        ":qp_problem_lib",
    ],
)

proto_library(
    name = "qp_problem_lib",
    srcs = ["qp_problem.proto"],
)

proto_library(
    name = "speed_bounds_decider_config_proto_lib",
    srcs = ["speed_bounds_decider_config.proto"],
)

cc_proto_library(
    name = "speed_bounds_decider_config_proto",
    deps = [
        ":speed_bounds_decider_config_proto_lib",
    ],
)

proto_library(
    name = "st_bounds_decider_config_proto_lib",
    srcs = ["st_bounds_decider_config.proto"],
)

cc_proto_library(
    name = "st_bounds_decider_config_proto",
    deps = [
        ":st_bounds_decider_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_fallback_decider_config_proto_lib",
    srcs = ["open_space_fallback_decider_config.proto"],
)

cc_proto_library(
    name = "open_space_fallback_decider_config_proto",
    deps = [
        ":open_space_fallback_decider_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_roi_decider_config_proto_lib",
    srcs = ["open_space_roi_decider_config.proto"],
)

cc_proto_library(
    name = "open_space_roi_decider_config_proto",
    deps = [
        ":open_space_roi_decider_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_trajectory_provider_config_proto_lib",
    srcs = ["open_space_trajectory_provider_config.proto"],
    deps = [
        ":planner_open_space_config_proto_lib",
    ],
)

cc_proto_library(
    name = "open_space_trajectory_provider_config_proto",
    deps = [
        ":open_space_trajectory_provider_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_trajectory_partition_config_proto_lib",
    srcs = ["open_space_trajectory_partition_config.proto"],
)

cc_proto_library(
    name = "open_space_trajectory_partition_config_proto",
    deps = [
        ":open_space_trajectory_partition_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_pre_stop_decider_config_proto_lib",
    srcs = ["open_space_pre_stop_decider_config.proto"],
)

cc_proto_library(
    name = "open_space_pre_stop_decider_config_proto",
    deps = [
        ":open_space_pre_stop_decider_config_proto_lib",
    ],
)

cc_proto_library(
    name = "rule_based_stop_decider_config_proto",
    deps = [
        ":rule_based_stop_decider_config_proto_lib",
    ],
)

proto_library(
    name = "rule_based_stop_decider_config_proto_lib",
    srcs = ["rule_based_stop_decider_config.proto"],
)

cc_proto_library(
    name = "cos_theta_smoother_config_proto",
    deps = [
        ":cos_theta_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "cos_theta_smoother_config_proto_lib",
    srcs = ["cos_theta_smoother_config.proto"],
)

cc_proto_library(
    name = "fem_pos_deviation_smoother_config_proto",
    deps = [
        ":fem_pos_deviation_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "fem_pos_deviation_smoother_config_proto_lib",
    srcs = ["fem_pos_deviation_smoother_config.proto"],
)

cc_proto_library(
    name = "ipopt_return_status_proto",
    deps = [
        ":ipopt_return_status_proto_lib",
    ],
)

proto_library(
    name = "ipopt_return_status_proto_lib",
    srcs = ["ipopt_return_status.proto"],
)

cc_proto_library(
    name = "learning_data_proto",
    deps = [
        ":learning_data_proto_lib",
    ],
)

proto_library(
    name = "learning_data_proto_lib",
    srcs = ["learning_data.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/map/proto:map_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:feature_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
    ],
)

py_proto(
    name = "path_bounds_decider_config_pb2",
    src = "path_bounds_decider_config.proto",
)

py_proto(
    name = "path_reuse_decider_config_pb2",
    src = "path_reuse_decider_config.proto",
)

py_proto(
    name = "path_decider_config_pb2",
    src = "path_decider_config.proto",
)

py_proto(
    name = "open_space_trajectory_partition_config_pb2",
    src = "open_space_trajectory_partition_config.proto",
)

py_proto(
    name = "pad_msg_pb2",
    src = "pad_msg.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "lattice_structure_pb2",
    src = "lattice_structure.proto",
)

py_proto(
    name = "rule_based_stop_decider_config_pb2",
    src = "rule_based_stop_decider_config.proto",
)

py_proto(
    name = "planning_stats_pb2",
    src = "planning_stats.proto",
)

py_proto(
    name = "decision_pb2",
    src = "decision.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:vehicle_signal_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "creep_decider_config_pb2",
    src = "creep_decider_config.proto",
)

py_proto(
    name = "spiral_curve_config_pb2",
    src = "spiral_curve_config.proto",
)

py_proto(
    name = "st_drivable_boundary_pb2",
    src = "st_drivable_boundary.proto",
)

py_proto(
    name = "qp_problem_pb2",
    src = "qp_problem.proto",
)

py_proto(
    name = "path_assessment_decider_config_pb2",
    src = "path_assessment_decider_config.proto",
)

py_proto(
    name = "piecewise_jerk_path_config_pb2",
    src = "piecewise_jerk_path_config.proto",
)

py_proto(
    name = "open_space_fallback_decider_config_pb2",
    src = "open_space_fallback_decider_config.proto",
)

py_proto(
    name = "planning_pb2",
    src = "planning.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:drive_state_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/map/proto:map_id_pb2",
        "//modules/planning/proto:decision_pb2",
        "//modules/planning/proto:planning_internal_pb2",
    ],
)

py_proto(
    name = "sl_boundary_pb2",
    src = "sl_boundary.proto",
    deps = [
        "//modules/common/proto:pnc_point_pb2",
    ],
)

py_proto(
    name = "cos_theta_smoother_config_pb2",
    src = "cos_theta_smoother_config.proto",
)

py_proto(
    name = "lattice_sampling_config_pb2",
    src = "lattice_sampling_config.proto",
)

py_proto(
    name = "reference_line_smoother_config_pb2",
    src = "reference_line_smoother_config.proto",
    deps = [
        "//modules/planning/proto:cos_theta_smoother_config_pb2",
        "//modules/planning/proto:fem_pos_deviation_smoother_config_pb2",
    ],
)

py_proto(
    name = "planning_status_pb2",
    src = "planning_status.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/planning/proto:planning_config_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "navi_path_decider_config_pb2",
    src = "navi_path_decider_config.proto",
)

py_proto(
    name = "ipopt_return_status_pb2",
    src = "ipopt_return_status.proto",
)

py_proto(
    name = "traffic_rule_config_pb2",
    src = "traffic_rule_config.proto",
)

py_proto(
    name = "auto_tuning_model_input_pb2",
    src = "auto_tuning_model_input.proto",
)

py_proto(
    name = "planning_internal_pb2",
    src = "planning_internal.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/dreamview/proto:chart_pb2",
        "//modules/localization/proto:localization_pb2",
        "//modules/map/relative_map/proto:navigation_pb2",
        "//modules/perception/proto:traffic_light_detection_pb2",
        "//modules/planning/proto:decision_pb2",
        "//modules/planning/proto:planning_config_pb2",
        "//modules/planning/proto:sl_boundary_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "open_space_roi_decider_config_pb2",
    src = "open_space_roi_decider_config.proto",
)

py_proto(
    name = "learning_data_pb2",
    src = "learning_data.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/perception/proto:traffic_light_detection_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "speed_bounds_decider_config_pb2",
    src = "speed_bounds_decider_config.proto",
)

py_proto(
    name = "planner_open_space_config_pb2",
    src = "planner_open_space_config.proto",
    deps = [
        "//modules/planning/proto:fem_pos_deviation_smoother_config_pb2",
        "//modules/planning/proto:piecewise_jerk_speed_config_pb2",
    ],
)

py_proto(
    name = "path_lane_borrow_decider_config_pb2",
    src = "path_lane_borrow_decider_config.proto",
)

py_proto(
    name = "open_space_pre_stop_decider_config_pb2",
    src = "open_space_pre_stop_decider_config.proto",
)

py_proto(
    name = "fem_pos_deviation_smoother_config_pb2",
    src = "fem_pos_deviation_smoother_config.proto",
)

py_proto(
    name = "navi_speed_decider_config_pb2",
    src = "navi_speed_decider_config.proto",
)

py_proto(
    name = "lane_change_decider_config_pb2",
    src = "lane_change_decider_config.proto",
)

py_proto(
    name = "open_space_trajectory_provider_config_pb2",
    src = "open_space_trajectory_provider_config.proto",
    deps = [
        "//modules/planning/proto:planner_open_space_config_pb2",
    ],
)

py_proto(
    name = "navi_obstacle_decider_config_pb2",
    src = "navi_obstacle_decider_config.proto",
)

py_proto(
    name = "st_bounds_decider_config_pb2",
    src = "st_bounds_decider_config.proto",
)

py_proto(
    name = "piecewise_jerk_nonlinear_speed_config_pb2",
    src = "piecewise_jerk_nonlinear_speed_config.proto",
)

py_proto(
    name = "planning_config_pb2",
    src = "planning_config.proto",
    deps = [
        "//modules/planning/proto:creep_decider_config_pb2",
        "//modules/planning/proto:dp_st_speed_config_pb2",
        "//modules/planning/proto:lane_change_decider_config_pb2",
        "//modules/planning/proto:navi_obstacle_decider_config_pb2",
        "//modules/planning/proto:navi_path_decider_config_pb2",
        "//modules/planning/proto:navi_speed_decider_config_pb2",
        "//modules/planning/proto:open_space_fallback_decider_config_pb2",
        "//modules/planning/proto:open_space_pre_stop_decider_config_pb2",
        "//modules/planning/proto:open_space_roi_decider_config_pb2",
        "//modules/planning/proto:open_space_trajectory_partition_config_pb2",
        "//modules/planning/proto:open_space_trajectory_provider_config_pb2",
        "//modules/planning/proto:path_assessment_decider_config_pb2",
        "//modules/planning/proto:path_bounds_decider_config_pb2",
        "//modules/planning/proto:path_decider_config_pb2",
        "//modules/planning/proto:path_lane_borrow_decider_config_pb2",
        "//modules/planning/proto:path_reuse_decider_config_pb2",
        "//modules/planning/proto:piecewise_jerk_nonlinear_speed_config_pb2",
        "//modules/planning/proto:piecewise_jerk_path_config_pb2",
        "//modules/planning/proto:piecewise_jerk_speed_config_pb2",
        "//modules/planning/proto:rule_based_stop_decider_config_pb2",
        "//modules/planning/proto:speed_bounds_decider_config_pb2",
        "//modules/planning/proto:st_bounds_decider_config_pb2",
    ],
)

py_proto(
    name = "dp_st_speed_config_pb2",
    src = "dp_st_speed_config.proto",
)

py_proto(
    name = "piecewise_jerk_speed_config_pb2",
    src = "piecewise_jerk_speed_config.proto",
)

py_proto(
    name = "auto_tuning_raw_feature_pb2",
    src = "auto_tuning_raw_feature.proto",
    deps = [
        "//modules/common/proto:pnc_point_pb2",
    ],
)
