load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "bitmap2d",
    srcs = ["bitmap2d.cc"],
    hdrs = ["bitmap2d.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/lidar/common:lidar_log",
        "@boost",
        "@eigen",
    ],
)

cyber_plugin_description(
    name = "plugin_libhdmap_roi_filter_lib_description",
    description = ":plugins.xml",
    plugin = ":libhdmap_roi_filter.so",
)

cc_binary(
    name = "libhdmap_roi_filter.so",
    linkshared = True,
    linkstatic = True,
    copts = PERCEPTION_COPTS,
    deps = [":hdmap_roi_filter_lib"],
)

cc_library(
    name = "hdmap_roi_filter_lib",
    srcs = ["hdmap_roi_filter.cc"],
    hdrs = ["hdmap_roi_filter.h"],
    deps = [
        ":bitmap2d",
        ":polygon_mask",
        ":polygon_scan_cvter",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/base:point_cloud",
        "//modules/perception/common/lidar/common:lidar_point_label",
        "//modules/perception/common/lidar/scene_manager",
        "//modules/perception/common/lidar/scene_manager/ground_service:ground_service",
        "//modules/perception/common/lidar/scene_manager/roi_service:roi_service",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/pointcloud_map_based_roi/interface:base_roi_filter",
        "//modules/perception/pointcloud_map_based_roi/roi_filter/hdmap_roi_filter/proto:hdmap_roi_filter_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
        "@eigen",
    ],
    alwayslink = True,
)

#cc_test(
#    name = "hdmap_roi_filter_test",
#    size = "small",
#    srcs = ["hdmap_roi_filter_test.cc"],
#    deps = [
#        ":hdmap_roi_filter",
#        "@com_google_googletest//:gtest_main",
#    ],
#)

cc_library(
    name = "polygon_mask",
    hdrs = ["polygon_mask.h"],
    deps = [
        ":bitmap2d",
        ":polygon_scan_cvter",
        "//modules/perception/common/lidar/common:lidar_log",
        "@eigen",
    ],
)

cc_library(
    name = "polygon_scan_cvter",
    hdrs = ["polygon_scan_cvter.h"],
    deps = [
        "//modules/perception/common/lidar/common:lidar_log",
        "@eigen",
    ],
)

apollo_package()
cpplint()
