load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//cyber",
        "//modules/perception/base:object",
        "@eigen",
    ],
)

cc_library(
    name = "type_fusion_interface",
    hdrs = ["type_fusion_interface.h"],
    deps = [
        "//modules/perception/base:object",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common",
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
        "//modules/perception/base:object",
        "//modules/perception/base:point_cloud",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common:object_sequence",
        "//modules/perception/proto:ccrf_type_fusion_config_proto",
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
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/lidar/common:object_sequence",
        "//modules/perception/lidar/lib/interface:base_classifier",
        "//modules/perception/proto:fused_classifier_config_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "fused_classifier_test",
    srcs = ["fused_classifier_test.cc"],
    deps = [
        ":fused_classifier",
        ":util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
