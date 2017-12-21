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
        "decision.proto",
        "planning.proto",
        "planning_config.proto",
        "planning_internal.proto",
        "planning_stats.proto",
        "qp_st_speed_config.proto",
        "sl_boundary.proto",
    ],
    deps = [
        ":dp_poly_path_config_proto_lib",
        ":dp_st_speed_config_proto_lib",
        ":qp_spline_path_config_proto_lib",
        ":qp_spline_reference_line_smoother_config_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:common_proto_lib",
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
    deps =[
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
    deps =[
        ":lattice_sampling_config_proto_lib",
         "//modules/common/proto:pnc_point_proto_lib",
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
        "st_boundary_config.proto",
    ],
)

cc_proto_library(
    name = "qp_spline_path_config_proto",
    deps = [
        ":qp_spline_path_config_proto_lib",
    ],
)

proto_library(
    name = "qp_spline_path_config_proto_lib",
    srcs = [
        "qp_spline_path_config.proto",
    ],
)

proto_library(
    name = "st_boundary_config_proto_lib",
    srcs = [
        "st_boundary_config.proto",
    ],
)

cc_proto_library(
    name = "qp_spline_reference_line_smoother_config_proto",
    deps = [
        ":qp_spline_reference_line_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "qp_spline_reference_line_smoother_config_proto_lib",
    srcs = [
        "qp_spline_reference_line_smoother_config.proto",
    ],
)

cpplint()
