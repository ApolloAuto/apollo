load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_state",
    srcs = [
        "vehicle_state.cc",
    ],
    hdrs = [
        "vehicle_state.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/math:quaternion",
        "//modules/localization/common:localization_common",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "vehicle_state_test",
    size = "small",
    srcs = [
        "vehicle_state_test.cc",
    ],
    data = [
        "//modules/canbus:canbus_testdata",
        "//modules/localization:localization_testdata",
    ],
    deps = [
        ":vehicle_state",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
