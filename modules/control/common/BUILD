load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_gflags",
    srcs = [
        "control_gflags.cc",
    ],
    hdrs = [
        "control_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "base_types",
    hdrs = [
        "base_types.h",
    ],
    deps = [
        ":definitions",
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "definitions",
    srcs = [],
    hdrs = [
        "definitions.h",
    ],
    deps = [
        "//modules/common/proto:error_code_proto",
        "//modules/common/status",
    ],
)

cc_library(
    name = "hysteresis_filter",
    srcs = [
        "hysteresis_filter.cc",
    ],
    hdrs = [
        "hysteresis_filter.h",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = [
        "interpolation_2d.cc",
    ],
    hdrs = [
        "interpolation_2d.h",
    ],
    deps = [
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = [
        "pid_controller.cc",
    ],
    hdrs = [
        "pid_controller.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "trajectory_analyzer",
    srcs = [
        "trajectory_analyzer.cc",
    ],
    hdrs = [
        "trajectory_analyzer.h",
    ],
    deps = [
        ":base_types",
        "//modules/common:log",
        "//modules/common/math:linear_interpolation",
        "//modules/common/math:search",
        "//modules/common/vehicle_state",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "common",
    deps = [
        ":base_types",
        ":control_gflags",
        ":definitions",
        ":hysteresis_filter",
        ":interpolation_2d",
        ":pid_controller",
        ":trajectory_analyzer",
    ],
)

cc_test(
    name = "hysteresis_filter_test",
    size = "small",
    srcs = [
        "hysteresis_filter_test.cc",
    ],
    deps = [
        ":hysteresis_filter",
        "@gtest//:main",
    ],
)

cc_test(
    name = "interpolation_2d_test",
    size = "small",
    srcs = [
        "interpolation_2d_test.cc",
    ],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":interpolation_2d",
        "//modules/common:log",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pid_controller_test",
    size = "small",
    srcs = [
        "pid_controller_test.cc",
    ],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":pid_controller",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "trajectory_analyzer_test",
    size = "small",
    srcs = [
        "trajectory_analyzer_test.cc",
    ],
    deps = [
        ":trajectory_analyzer",
        "//modules/common/time",
        "@gtest//:main",
    ],
)

cpplint()
