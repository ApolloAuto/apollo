load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bitmap2d",
    srcs = ["bitmap2d.cc"],
    hdrs = ["bitmap2d.h"],
    deps = [
        "//cyber",
        "//modules/perception/lidar/common:lidar_log",
        "@eigen",
    ],
)

cc_library(
    name = "hdmap_roi_filter",
    srcs = ["hdmap_roi_filter.cc"],
    hdrs = ["hdmap_roi_filter.h"],
    deps = [
        ":bitmap2d",
        ":polygon_mask",
        ":polygon_scan_cvter",
        "//cyber",
        "//modules/perception/base:point_cloud",
        "//modules/perception/lidar/common:lidar_point_label",
        "//modules/perception/lidar/lib/interface:base_object_filter",
        "//modules/perception/lidar/lib/interface:base_roi_filter",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/proto:hdmap_roi_filter_proto",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
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
        "//modules/perception/lidar/common:lidar_log",
        "@eigen",
    ],
)

cc_library(
    name = "polygon_scan_cvter",
    hdrs = ["polygon_scan_cvter.h"],
    deps = [
        "//modules/perception/lidar/common:lidar_log",
        "@eigen",
    ],
)

cpplint()
