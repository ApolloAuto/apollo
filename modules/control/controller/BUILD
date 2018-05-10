load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controller_interface",
    srcs = [],
    hdrs = [
        "controller.h",
    ],
    deps = [],
)

cc_library(
    name = "lat_controller",
    srcs = [
        "lat_controller.cc",
    ],
    hdrs = [
        "lat_controller.h",
    ],
    deps = [
        ":controller_interface",
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/filters:mean_filter",
        "//modules/common/math:euler_angles_zxy",
        "//modules/common/math:geometry",
        "//modules/common/math:lqr",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_1d",
        "//modules/control/common:trajectory_analyzer",
        "//modules/control/proto:control_proto",
        "@eigen",
    ],
)

cc_library(
    name = "lon_controller",
    srcs = [
        "lon_controller.cc",
    ],
    hdrs = [
        "lon_controller.h",
    ],
    deps = [
        ":controller_interface",
        "//modules/common:log",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_2d",
        "//modules/control/common:pid_controller",
        "//modules/control/common:trajectory_analyzer",
        "//modules/localization/common:localization_common",
    ],
)

cc_library(
    name = "mpc_controller",
    srcs = [
        "mpc_controller.cc",
    ],
    hdrs = [
        "mpc_controller.h",
    ],
    deps = [
        ":controller_interface",
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/filters:mean_filter",
        "//modules/common/math:euler_angles_zxy",
        "//modules/common/math:geometry",
        "//modules/common/math:lqr",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_1d",
        "//modules/control/common:interpolation_2d",
        "//modules/control/common:trajectory_analyzer",
        "//modules/control/proto:control_proto",
        "@eigen",
    ],
)

cc_library(
    name = "controller",
    deps = [
        ":controller_agent",
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        ":mpc_controller",
        "@eigen",
    ],
)

cc_library(
    name = "controller_agent",
    srcs = [
        "controller_agent.cc",
    ],
    hdrs = [
        "controller_agent.h",
    ],
    deps = [
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        ":mpc_controller",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util:factory",
        "//modules/control/proto:control_proto",
        "//modules/planning/proto:planning_proto",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "lon_controller_test",
    size = "small",
    srcs = [
        "lon_controller_test.cc",
    ],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":lon_controller",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_proto",
        "//modules/planning/proto:planning_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = [
        "lat_controller_test.cc",
    ],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":lat_controller",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_proto",
        "//modules/planning/proto:planning_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "mpc_controller_test",
    size = "small",
    srcs = [
        "mpc_controller_test.cc",
    ],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":mpc_controller",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_proto",
        "//modules/planning/proto:planning_proto",
        "@gtest//:main",
    ],
)

cpplint()
