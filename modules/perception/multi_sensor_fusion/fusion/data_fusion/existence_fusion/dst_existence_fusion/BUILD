load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dst_existence_fusion",
    srcs = ["dst_existence_fusion.cc"],
    hdrs = ["dst_existence_fusion.h"],
    deps = [
        "//modules/common/util",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/multi_sensor_fusion/base",
        "//modules/perception/multi_sensor_fusion/common:camera_util",
        "//modules/perception/multi_sensor_fusion/common:dst_evidence",
        "//modules/perception/multi_sensor_fusion/interface",
        "//modules/perception/multi_sensor_fusion/proto:dst_existence_fusion_config_cc_proto",
    ],
)

cc_test(
    name = "dst_existence_fusion_test",
    size = "small",
    srcs = ["dst_existence_fusion_test.cc"],
    deps = [
        ":dst_existence_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
