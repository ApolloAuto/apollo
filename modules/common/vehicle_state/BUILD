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
        "//cyber/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:geometry",
        "//modules/common/math:quaternion",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/localization/common:localization_common",
        "@eigen",
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
