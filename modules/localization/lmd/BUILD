load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lmd_localization",
    srcs = [
        "lmd_localization.cc",
    ],
    hdrs = [
        "lmd_localization.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/localization:localization_base",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:odometry_lane_marker_proto",
        "@gtest",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "lmd_localization_test",
    size = "small",
    srcs = [
        "lmd_localization_test.cc",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/lmd:lmd_localization",
        "@gtest//:main",
    ],
)

cpplint()
