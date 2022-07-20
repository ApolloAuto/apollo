load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvehicle_state_provider.so",
    srcs = [
        "vehicle_state_provider.cc",
        "vehicle_state_provider.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@eigen",
        "@com_google_absl//:absl",
    ],
)
cc_library(
    name = "vehicle_state_provider",
    srcs = ["libvehicle_state_provider.so"],
    hdrs = ["vehicle_state_provider.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "@eigen",
        "@com_google_absl//:absl",
    ],
)

cc_test(
    name = "vehicle_state_provider_test",
    size = "small",
    srcs = ["vehicle_state_provider_test.cc"],
    data = [
        "//modules/canbus:test_data",
        "//modules/localization:localization_testdata",
    ],
    deps = [
        ":vehicle_state_provider",
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/configs:config_gflags",
        "@eigen",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
