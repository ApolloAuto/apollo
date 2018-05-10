load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_spline_st_graph",
    srcs = [
        "qp_spline_st_graph.cc",
    ],
    hdrs = [
        "qp_spline_st_graph.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/math",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util",
        "//modules/planning/common:planning_util",
        "//modules/planning/math/smoothing_spline:spline_1d_generator",
        "//modules/planning/proto:qp_st_speed_config_proto",
        "//modules/planning/tasks/st_graph:st_boundary_mapper",
        "//modules/planning/tasks/st_graph:st_graph_data",
    ],
)

cc_library(
    name = "qp_piecewise_st_graph",
    srcs = [
        "qp_piecewise_st_graph.cc",
    ],
    hdrs = [
        "qp_piecewise_st_graph.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/math",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util",
        "//modules/planning/common:planning_util",
        "//modules/planning/math/smoothing_spline:piecewise_linear_generator",
        "//modules/planning/proto:qp_st_speed_config_proto",
        "//modules/planning/tasks/st_graph:st_boundary_mapper",
        "//modules/planning/tasks/st_graph:st_graph_data",
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
        ":qp_piecewise_st_graph",
        ":qp_spline_st_graph",
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/map/hdmap",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:speed_optimizer",
        "//modules/planning/tasks/st_graph:st_boundary_mapper",
    ],
)

cpplint()
