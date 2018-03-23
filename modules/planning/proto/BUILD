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
        "planning.proto",
    ],
    deps = [
        ":decision_proto_lib",
        ":planning_internal_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
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
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "dp_poly_path_config_proto",
    deps = [
        ":dp_poly_path_config_proto_lib",
    ],
)

proto_library(
    name = "dp_poly_path_config_proto_lib",
    srcs = [
        "dp_poly_path_config.proto",
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
    deps = [
        ":st_boundary_config_proto_lib",
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
        ":dp_poly_path_config_proto_lib",
        ":dp_st_speed_config_proto_lib",
        ":poly_st_speed_config_proto_lib",
        ":qp_spline_path_config_proto_lib",
        ":qp_st_speed_config_proto_lib",
        ":reference_line_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "planning_internal_proto_lib",
    srcs = [
        "planning_internal.proto",
    ],
    deps = [
        ":decision_proto_lib",
        ":sl_boundary_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
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

proto_library(
    name = "qp_spline_path_config_proto_lib",
    srcs = [
        "qp_spline_path_config.proto",
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
)

cc_proto_library(
    name = "qp_st_speed_config_proto",
    deps = [
        ":qp_st_speed_config_proto_lib",
    ],
)

proto_library(
    name = "qp_st_speed_config_proto_lib",
    srcs = [
        "qp_st_speed_config.proto",
    ],
    deps = [
        ":st_boundary_config_proto_lib",
    ],
)

cc_proto_library(
    name = "poly_st_speed_config_proto",
    deps = [
        ":poly_st_speed_config_proto_lib",
    ],
)

proto_library(
    name = "poly_st_speed_config_proto_lib",
    srcs = [
        "poly_st_speed_config.proto",
    ],
    deps = [
        ":st_boundary_config_proto_lib",
    ],
)

proto_library(
    name = "sl_boundary_proto_lib",
    srcs = [
        "sl_boundary.proto",
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
    name = "st_boundary_config_proto",
    deps = [
        ":st_boundary_config_proto_lib",
    ],
)

proto_library(
    name = "st_boundary_config_proto_lib",
    srcs = [
        "st_boundary_config.proto",
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
        ":decision_proto_lib",
        ":planning_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
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

cpplint()
