load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtk_localization",
    srcs = [
        "rtk_localization.cc",
    ],
    hdrs = [
        "rtk_localization.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/localization:localization_base",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "@gtest",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "rtk_localization_test",
    size = "small",
    srcs = [
        "rtk_localization_test.cc",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/rtk:rtk_localization",
        "@gtest//:main",
    ],
)

cpplint()
