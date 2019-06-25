load("//tools:cpplint.bzl", "cpplint")

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
    name = "decider_config_proto",
    deps = [
        ":decider_config_proto_lib",
    ],
)

proto_library(
    name = "decider_config_proto_lib",
    srcs = [
        "decider_config.proto",
    ],
    deps = [
    ],
)

cc_proto_library(
    name = "decision_proto",
    deps = [
        ":decision_proto_lib",
    ],
)

proto_library(
    name = "decision_proto_lib",
    srcs = [
        "decision.proto",
    ],
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
    srcs = [
        "dp_st_speed_config.proto",
    ],
)

cc_proto_library(
    name = "proceed_with_caution_speed_config_proto",
    deps = [
        ":proceed_with_caution_speed_config_proto_lib",
    ],
)

proto_library(
    name = "proceed_with_caution_speed_config_proto_lib",
    srcs = [
        "proceed_with_caution_speed_config.proto",
    ],
)

cc_proto_library(
    name = "navi_path_decider_config_proto",
    deps = [
        ":navi_path_decider_config_proto_lib",
    ],
)

proto_library(
    name = "navi_path_decider_config_proto_lib",
    srcs = [
        "navi_path_decider_config.proto",
    ],
)

cc_proto_library(
    name = "navi_speed_decider_config_proto",
    deps = [
        ":navi_speed_decider_config_proto_lib",
    ],
)

proto_library(
    name = "navi_speed_decider_config_proto_lib",
    srcs = [
        "navi_speed_decider_config.proto",
    ],
)

cc_proto_library(
    name = "navi_obstacle_decider_config_proto",
    deps = [
        ":navi_obstacle_decider_config_proto_lib",
    ],
)

proto_library(
    name = "navi_obstacle_decider_config_proto_lib",
    srcs = [
        "navi_obstacle_decider_config.proto",
    ],
)

cc_proto_library(
    name = "planning_config_proto",
    deps = [
        ":planning_config_proto_lib",
    ],
)

proto_library(
    name = "planning_config_proto_lib",
    srcs = [
        "planning_config.proto",
    ],
    deps = [
        ":decider_config_proto_lib",
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
        ":path_lane_borrow_decider_config_proto_lib",
        ":piecewise_jerk_path_config_proto_lib",
        ":piecewise_jerk_speed_config_proto_lib",
        ":planner_open_space_config_proto_lib",
        ":proceed_with_caution_speed_config_proto_lib",
        ":reference_line_smoother_config_proto_lib",
        ":rule_based_stop_decider_config_proto_lib",
        ":speed_bounds_decider_config_proto_lib",
    ],
)

proto_library(
    name = "planning_internal_proto_lib",
    srcs = [
        "planning_internal.proto",
    ],
    deps = [
        ":decision_proto_lib",
        ":planning_config_proto_lib",
        ":planning_status_proto_lib",
        ":sl_boundary_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
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
    srcs = [
        "planning_stats.proto",
    ],
)

cc_proto_library(
    name = "reference_line_smoother_config_proto",
    deps = [
        ":reference_line_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "reference_line_smoother_config_proto_lib",
    srcs = [
        "reference_line_smoother_config.proto",
    ],
    deps = [
        ":cos_theta_smoother_config_proto_lib",
        ":fem_pos_deviation_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "sl_boundary_proto_lib",
    srcs = [
        "sl_boundary.proto",
    ],
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
    srcs = [
        "traffic_rule_config.proto",
    ],
)

cc_proto_library(
    name = "piecewise_jerk_path_config_proto",
    deps = [
        ":piecewise_jerk_path_config_proto_lib",
    ],
)

proto_library(
    name = "piecewise_jerk_path_config_proto_lib",
    srcs = [
        "piecewise_jerk_path_config.proto",
    ],
)

cc_proto_library(
    name = "piecewise_jerk_speed_config_proto",
    deps = [
        ":piecewise_jerk_speed_config_proto_lib",
    ],
)

proto_library(
    name = "piecewise_jerk_speed_config_proto_lib",
    srcs = [
        "piecewise_jerk_speed_config.proto",
    ],
)

cc_proto_library(
    name = "path_bounds_decider_config_proto",
    deps = [
        ":path_bounds_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_bounds_decider_config_proto_lib",
    srcs = [
        "path_bounds_decider_config.proto",
    ],
)

cc_proto_library(
    name = "path_assessment_decider_config_proto",
    deps = [
        ":path_assessment_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_assessment_decider_config_proto_lib",
    srcs = [
        "path_assessment_decider_config.proto",
    ],
)

cc_proto_library(
    name = "path_lane_borrow_decider_config_proto",
    deps = [
        ":path_lane_borrow_decider_config_proto_lib",
    ],
)

proto_library(
    name = "path_lane_borrow_decider_config_proto_lib",
    srcs = [
        "path_lane_borrow_decider_config.proto",
    ],
)

cc_proto_library(
    name = "lane_change_decider_config_proto",
    deps = [
        ":lane_change_decider_config_proto_lib",
    ],
)

proto_library(
    name = "lane_change_decider_config_proto_lib",
    srcs = [
        "lane_change_decider_config.proto",
    ],
)

cc_proto_library(
    name = "spiral_curve_config_proto",
    deps = [
        ":spiral_curve_config_proto_lib",
    ],
)

proto_library(
    name = "spiral_curve_config_proto_lib",
    srcs = [
        "spiral_curve_config.proto",
    ],
)

cc_proto_library(
    name = "planning_status_proto",
    deps = [
        ":planning_status_proto_lib",
    ],
)

proto_library(
    name = "planning_status_proto_lib",
    srcs = [
        "planning_status.proto",
    ],
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
    srcs = [
        "lattice_sampling_config.proto",
    ],
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
    srcs = [
        "lattice_structure.proto",
    ],
    deps = [
        ":lattice_sampling_config_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

proto_library(
    name = "planner_open_space_config_proto_lib",
    srcs = [
        "planner_open_space_config.proto",
    ],
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
    srcs = [
        "auto_tuning_raw_feature.proto",
    ],
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
    srcs = [
        "auto_tuning_model_input.proto",
    ],
)

cc_proto_library(
    name = "qp_problem_proto",
    deps = [
        ":qp_problem_lib",
    ],
)

proto_library(
    name = "qp_problem_lib",
    srcs = [
        "qp_problem.proto",
    ],
)

proto_library(
    name = "speed_bounds_decider_config_proto_lib",
    srcs = [
        "speed_bounds_decider_config.proto",
    ],
)

cc_proto_library(
    name = "speed_bounds_decider_config_proto",
    deps = [
        ":speed_bounds_decider_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_fallback_decider_config_proto_lib",
    srcs = [
        "open_space_fallback_decider_config.proto",
    ],
)

cc_proto_library(
    name = "open_space_fallback_decider_config_proto",
    deps = [
        ":open_space_fallback_decider_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_roi_decider_config_proto_lib",
    srcs = [
        "open_space_roi_decider_config.proto",
    ],
)

cc_proto_library(
    name = "open_space_roi_decider_config_proto",
    deps = [
        ":open_space_roi_decider_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_trajectory_provider_config_proto_lib",
    srcs = [
        "open_space_trajectory_provider_config.proto",
    ],
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
    srcs = [
        "open_space_trajectory_partition_config.proto",
    ],
)

cc_proto_library(
    name = "open_space_trajectory_partition_config_proto",
    deps = [
        ":open_space_trajectory_partition_config_proto_lib",
    ],
)

proto_library(
    name = "open_space_pre_stop_decider_config_proto_lib",
    srcs = [
        "open_space_pre_stop_decider_config.proto",
    ],
)

cc_proto_library(
    name = "open_space_pre_stop_decider_config_proto",
    deps = [
        ":open_space_pre_stop_decider_config_proto_lib",
    ],
)

cc_proto_library(
    name = "path_decider_info_proto",
    deps = [
        ":path_decider_info_proto_lib",
    ],
)

proto_library(
    name = "path_decider_info_proto_lib",
    srcs = [
        "path_decider_info.proto",
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
    srcs = [
        "rule_based_stop_decider_config.proto",
    ],
)

cc_proto_library(
    name = "cos_theta_smoother_config_proto",
    deps = [
        ":cos_theta_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "cos_theta_smoother_config_proto_lib",
    srcs = [
        "cos_theta_smoother_config.proto",
    ],
)

cc_proto_library(
    name = "fem_pos_deviation_smoother_config_proto",
    deps = [
        ":fem_pos_deviation_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "fem_pos_deviation_smoother_config_proto_lib",
    srcs = [
        "fem_pos_deviation_smoother_config.proto",
    ],
)

cpplint()
