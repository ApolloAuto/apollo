load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "discretized_path",
    srcs = [
        "discretized_path.cc",
    ],
    hdrs = [
        "discretized_path.h",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:planning_util",
    ],
)

cc_test(
    name = "discretized_path_test",
    size = "small",
    srcs = [
        "discretized_path_test.cc",
    ],
    deps = [
        ":discretized_path",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "frenet_frame_path",
    srcs = [
        "frenet_frame_path.cc",
    ],
    hdrs = [
        "frenet_frame_path.h",
    ],
    deps = [
        "//modules/common/math:linear_interpolation",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:planning_util",
    ],
)

cc_test(
    name = "frenet_frame_path_test",
    size = "small",
    srcs = [
        "frenet_frame_path_test.cc",
    ],
    deps = [
        ":frenet_frame_path",
        "@gtest//:main",
    ],
)

cc_library(
    name = "path_data",
    srcs = [
        "path_data.cc",
    ],
    hdrs = [
        "path_data.h",
    ],
    deps = [
        ":discretized_path",
        ":frenet_frame_path",
        "//modules/common/math:cartesian_frenet_conversion",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/reference_line",
    ],
)

cpplint()
