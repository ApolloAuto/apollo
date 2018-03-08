load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_spline_path",
    srcs = [
        "qp_frenet_frame.cc",
        "qp_spline_path_generator.cc",
        "qp_spline_path_optimizer.cc",
    ],
    hdrs = [
        "qp_frenet_frame.h",
        "qp_spline_path_generator.h",
        "qp_spline_path_optimizer.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/math:cartesian_frenet_conversion",
        "//modules/common/math/qp_solver",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/math:polynomial_xd",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/math/smoothing_spline:spline_1d_constraint",
        "//modules/planning/math/smoothing_spline:spline_1d_generator",
        "//modules/planning/math/smoothing_spline:spline_1d_kernel",
        "//modules/planning/math/smoothing_spline:spline_1d_seg",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/tasks:path_optimizer",
        "@eigen//:eigen",
    ],
)

cpplint()
