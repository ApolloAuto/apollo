load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base:object",
        "@eigen",
    ],
)

cc_library(
    name = "type_fusion_interface",
    hdrs = ["type_fusion_interface.h"],
    deps = [
        "//modules/perception/common/base:object",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/lidar/common",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

cc_library(
    name = "ccrf_type_fusion",
    srcs = ["ccrf_type_fusion.cc"],
    hdrs = ["ccrf_type_fusion.h"],
    deps = [
        ":type_fusion_interface",
        ":util",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common:util",
        "//modules/perception/common/base:object",
        "//modules/perception/common/base:point_cloud",
        "//modules/perception/common/lidar/common:object_sequence",
        "//modules/perception/lidar_tracking/classifier/fused_classifier/proto:ccrf_type_fusion_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "fused_classifier",
    srcs = ["fused_classifier.cc"],
    hdrs = ["fused_classifier.h"],
    deps = [
        ":type_fusion_interface",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/lidar/common:lidar_frame",
        "//modules/perception/common/lidar/common:object_sequence",
        "//modules/perception/lidar_tracking/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/lidar_tracking/interface:base_classifier",
        "//modules/perception/lidar_tracking/classifier/fused_classifier/proto:fused_classifier_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "fused_classifier_test",
    srcs = ["fused_classifier_test.cc"],
    deps = [
        ":fused_classifier",
        ":util",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
