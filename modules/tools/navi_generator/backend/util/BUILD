load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_smoother",
    srcs = [
        "trajectory_smoother.cc",
    ],
    hdrs = [
        "trajectory_smoother.h",
    ],
    deps = [
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/reference_line:spiral_reference_line_smoother",
        "//modules/tools/navi_generator/backend/common:navi_generator_gflags",
        "//modules/tools/navi_generator/proto:trajectory_util_config_proto",

    ],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cc_test(
    name = "trajectory_smoother_test",
    size = "small",
    srcs = [
        "trajectory_smoother_test.cc",
    ], 
    data = [        
        ":testdata",
    ],   
    deps = [
        ":trajectory_smoother",
        "@gtest//:main",
    ],
)

cpplint()
