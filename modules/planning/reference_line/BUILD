load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reference_line",
    srcs = [
        "reference_line.cc",
        "reference_point.cc",
    ],
    hdrs = [
        "reference_line.h",
        "reference_point.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util:string_util",
        "//modules/map/pnc_map:path",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "reference_line_smoother",
    srcs = [
    ],
    hdrs = [
        "reference_line_smoother.h",
    ],
    deps = [
        "//modules/planning/proto:reference_line_smoother_config_proto",
    ],
)

cc_library(
    name = "qp_spline_reference_line_smoother",
    srcs = [
        "qp_spline_reference_line_smoother.cc",
    ],
    hdrs = [
        "qp_spline_reference_line_smoother.h",
    ],
    deps = [
        ":reference_line",
        ":reference_line_smoother",
        "//modules/planning/math:curve_math",
        "//modules/planning/math:polynomial_xd",
        "//modules/planning/math/smoothing_spline:spline_2d_solver",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:reference_line_smoother_config_proto",
        "@eigen",
    ],
)

cc_library(
    name = "spiral_reference_line_smoother",
    srcs = [
        "spiral_problem_interface.cc",
        "spiral_reference_line_smoother.cc",
    ],
    hdrs = [
        "spiral_problem_interface.h",
        "spiral_reference_line_smoother.h",
    ],
    deps = [
        ":reference_line",
        ":reference_line_smoother",
        "//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math/curve1d:quintic_spiral_path",
        "//modules/planning/math/smoothing_spline:spline_2d_solver",
        "//modules/planning/proto:planning_proto",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "cos_theta_reference_line_smoother",
    srcs = [
        "cos_theta_problem_interface.cc",
        "cos_theta_reference_line_smoother.cc",
    ],
    hdrs = [
        "cos_theta_problem_interface.h",
        "cos_theta_reference_line_smoother.h",
    ],
    deps = [
        ":qp_spline_reference_line_smoother",
        ":reference_line",
        ":reference_line_smoother",
        "//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math:curve_math",
        "//modules/planning/proto:planning_proto",
        "@eigen",
        "@ipopt",
    ],
)

cc_test(
    name = "qp_spline_reference_line_smoother_test",
    size = "small",
    srcs = [
        "qp_spline_reference_line_smoother_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":qp_spline_reference_line_smoother",
        "//modules/map/hdmap",
        "@gtest//:main",
    ],
)

cc_test(
    name = "cos_theta_reference_line_smoother_test",
    size = "small",
    srcs = [
        "cos_theta_reference_line_smoother_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":cos_theta_reference_line_smoother",
        "//modules/map/hdmap",
        "@gtest//:main",
    ],
)

cc_library(
    name = "reference_line_provider",
    srcs = [
        "reference_line_provider.cc",
    ],
    hdrs = [
        "reference_line_provider.h",
    ],
    deps = [
        ":cos_theta_reference_line_smoother",
        ":qp_spline_reference_line_smoother",
        ":reference_line",
        ":spiral_reference_line_smoother",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/pnc_map",
        "//modules/planning/common:indexed_queue",
        "//modules/planning/common:planning_context",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_status_proto",
    ],
)

cc_binary(
    name = "smoother_util",
    srcs = [
        "smoother_util.cc",
    ],
    deps = [
        ":cos_theta_reference_line_smoother",
        ":qp_spline_reference_line_smoother",
        ":reference_line",
        ":spiral_reference_line_smoother",
        "//modules/planning/proto:planning_config_proto",
    ],
)

cc_binary(
    name = "spiral_smoother_util",
    srcs = [
        "spiral_smoother_util.cc",
    ],
    deps = [
        ":spiral_reference_line_smoother",
        "//modules/planning/proto:planning_config_proto",
    ],
)

cpplint()
