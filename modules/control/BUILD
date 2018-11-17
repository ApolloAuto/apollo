load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_pad_terminal",
    deps = [
        "//modules/control/common",
    ],
)

cc_library(
    name = "control_lib",
    srcs = [
        "control.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/control/common",
        "//modules/control/controller",
        "//modules/control/proto:control_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "control",
    srcs = ["main.cc"],
    deps = [
        ":control_lib",
        "//external:gflags",
        "//modules/control/common",
    ],
)

cc_test(
    name = "control_test",
    size = "small",
    srcs = [
        "control_test.cc",
    ],
    data = ["//modules/control:control_testdata"],
    deps = [
        ":control_lib",
        "@gtest//:main",
    ],
)

filegroup(
    name = "control_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
