load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_spline_st_boundary_mapper",
    srcs = [
        "qp_spline_st_boundary_mapper.cc",
    ],
    hdrs = [
        "qp_spline_st_boundary_mapper.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/common/vehicle_state",
        "//modules/planning/optimizer/st_graph:st_boundary_mapper",
    ],
)

cc_library(
    name = "qp_spline_st_graph",
    srcs = [
        "qp_spline_st_graph.cc",
    ],
    hdrs = [
        "qp_spline_st_graph.h",
    ],
    deps = [
        ":qp_spline_st_boundary_mapper",
        "//modules/common:log",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/math",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:path_point_proto",
        "//modules/common/util",
        "//modules/planning/math/smoothing_spline:spline_1d_generator",
        "//modules/planning/optimizer/st_graph:st_graph_data",
        "//modules/planning/util:planning_util",
    ],
)

cc_library(
    name = "qp_spline_st_speed_optimizer",
    srcs = [
        "qp_spline_st_speed_optimizer.cc",
    ],
    hdrs = [
        "qp_spline_st_speed_optimizer.h",
    ],
    deps = [
        ":qp_spline_st_boundary_mapper",
        ":qp_spline_st_graph",
        "//modules/common:log",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:path_point_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/map/hdmap",
        "//modules/planning/optimizer:speed_optimizer",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
