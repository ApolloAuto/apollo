load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "vehicle_state_provider",
    srcs = ["vehicle_state_provider.cc"],
    hdrs = ["vehicle_state_provider.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "vehicle_state_provider_test",
    size = "small",
    srcs = ["vehicle_state_provider_test.cc"],
    data = [
        ":testdata",
    ],
    deps = [
        ":vehicle_state_provider",
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/*",
    ]),
)

apollo_package()

cpplint()
