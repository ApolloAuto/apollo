load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hdmap_roi_filter",
    srcs = [
        "bitmap2d.cc",
        "hdmap_roi_filter.cc",
        "polygon_mask.cc",
        "polygon_scan_converter.cc",
    ],
    hdrs = [
        "bitmap2d.h",
        "hdmap_roi_filter.h",
        "polygon_mask.h",
        "polygon_scan_converter.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/onboard:hdmapinput",
        "@eigen",
        "@pcl",
    ],
)

cc_test(
    name = "hdmap_roi_filter_test",
    size = "small",
    srcs = [
        "hdmap_roi_filter_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "@gtest",
        "@gtest//:main",
    ],
)

cpplint()
