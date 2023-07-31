load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dst_type_fusion",
    srcs = ["dst_type_fusion.cc"],
    hdrs = ["dst_type_fusion.h"],
    deps = [
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/multi_sensor_fusion/base:scene",
        "//modules/perception/multi_sensor_fusion/base:sensor_data_manager",
        "//modules/perception/multi_sensor_fusion/base:track",
        "//modules/perception/multi_sensor_fusion/common:camera_util",
        "//modules/perception/multi_sensor_fusion/common:dst_evidence",
        "//modules/perception/multi_sensor_fusion/interface",
        "//modules/perception/multi_sensor_fusion/proto:dst_type_fusion_config_cc_proto",
        "@com_google_absl//:absl",
    ],
)

cc_test(
    name = "dst_type_fusion_test",
    size = "small",
    srcs = ["dst_type_fusion_test.cc"],
    deps = [
        ":dst_type_fusion",
        "//modules/perception/common/base:base_type",
        "//modules/perception/common/base:frame",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
