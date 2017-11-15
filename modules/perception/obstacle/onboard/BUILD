load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_hdmapinput",
    srcs = ["hdmap_input.cc"],
    hdrs = ["hdmap_input.h"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "@eigen//:eigen",
        "@gtest//:gtest",
    ],
)

cc_library(
    name = "perception_obstacle_lidar_process",
    srcs = ["lidar_process.cc"],
    hdrs = ["lidar_process.h"],
    deps = [
        ":perception_obstacle_hdmapinput",
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/lidar/dummy:perception_obstacle_lidar_dummy",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "//modules/perception/obstacle/lidar/object_builder/min_box:perception_obstacle_lidar_object_builder_min_box",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:perception_obstacle_lidar_segmention_cnnseg",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker:perception_obstacle_lidar_tracker_hm_tracker",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer:perception_obstacle_lidar_opengl_visualizer",
        "@eigen//:eigen",
        "@gtest//:gtest",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "perception_obstacle_lidar_subnode",
    srcs = [
        "lidar_process_subnode.cc",
    ],
    hdrs = [
        "lidar_process_subnode.h",
        "object_shared_data.h",
    ],
    deps = [
        ":perception_obstacle_hdmapinput",
        "//modules/common",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/lidar/dummy:perception_obstacle_lidar_dummy",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "//modules/perception/obstacle/lidar/object_builder/min_box:perception_obstacle_lidar_object_builder_min_box",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:perception_obstacle_lidar_segmention_cnnseg",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker:perception_obstacle_lidar_tracker_hm_tracker",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer:perception_obstacle_lidar_opengl_visualizer",
        "@eigen//:eigen",
        "@gtest//:gtest",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "perception_obstacle_radar_subnode",
    srcs = [
        "radar_process_subnode.cc",
    ],
    hdrs = [
        "radar_process_subnode.h",
        "object_shared_data.h",
    ],
    deps = [
        ":perception_obstacle_hdmapinput",
        "//modules/common",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/radar/dummy:perception_obstacle_radar_dummy",
        "//modules/perception/obstacle/radar/modest:perception_obstacle_radar_modest_modest_detector",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer:perception_obstacle_lidar_opengl_visualizer",
        "@eigen//:eigen",
        "@gtest//:gtest",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "perception_obstacle_fusion_subnode",
    srcs = [
        "fusion_subnode.cc",
    ],
    hdrs = [
        "fusion_subnode.h",
        "object_shared_data.h",
    ],
    deps = [
        ":perception_obstacle_hdmapinput",
        "//modules/common",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/fusion/probabilistic_fusion:perception_obstacle_fusion_probabilistic_fusion",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer:perception_obstacle_lidar_opengl_visualizer",
        "@eigen//:eigen",
        "@gtest//:gtest",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "perception_obstacle_hdmap_input_test",
    size = "small",
    srcs = [
        "hdmap_input_test.cc",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/perception:perception_data",
    ],
    deps = [
        ":perception_obstacle_hdmapinput",
        "//modules/common/configs:config_gflags",
        "//modules/perception/common:perception_common",
        "@gtest//:main",
    ],
)

cc_test(
    name = "perception_obstacle_lidar_process_test",
    size = "small",
    srcs = [
        "lidar_process_test.cc",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/perception:perception_data",
    ],
    deps = [
        ":perception_obstacle_hdmapinput",
        ":perception_obstacle_lidar_process",
        "//modules/perception/common:perception_common",
        "@gtest//:main",
    ],
)

cpplint()
