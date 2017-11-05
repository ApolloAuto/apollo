load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_state_provider",
    srcs = [
        "vehicle_state_provider.cc",
    ],
    hdrs = [
        "vehicle_state_provider.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:box2d",
        "//modules/common/math:quaternion",
        "//modules/common/status",
        "//modules/localization/common:localization_common",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "vehicle_state_provider_test",
    size = "small",
    srcs = [
        "vehicle_state_provider_test.cc",
    ],
    data = [
        "//modules/canbus:canbus_testdata",
        "//modules/localization:localization_testdata",
    ],
    deps = [
        ":vehicle_state_provider",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
