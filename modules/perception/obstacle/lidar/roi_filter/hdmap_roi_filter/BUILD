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
        "//modules/common:common",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/obstacle/onboard:perception_obstacle_hdmapinput",
        "//external:gflags",
        "@eigen//:eigen",
        "@pcl//:pcl",
    ]
)
cc_test(
    name = "hdmap_roi_filter_test",
    size = "small",
    srcs = [
        "hdmap_roi_filter_test.cc"
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter:hdmap_roi_filter",
        "//external:gflags",
        "@gtest//:gtest",
        "@gtest//:main",
    ],
    data = [
        "//modules/perception:perception_data",
    ]
)



cpplint()
