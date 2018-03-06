load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hdmapinput",
    srcs = ["hdmap_input.cc"],
    hdrs = ["hdmap_input.h"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_process",
    srcs = ["lidar_process.cc"],
    hdrs = ["lidar_process.h"],
    deps = [
        ":hdmapinput",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/lidar/dummy",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker",
        "//modules/perception/obstacle/lidar/type_fuser/sequence_type_fuser",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/radar/dummy",
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
        ":hdmapinput",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/lidar/dummy",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker",
        "//modules/perception/obstacle/lidar/type_fuser/sequence_type_fuser",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/onboard:perception_onboard",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "perception_obstacle_radar_subnode",
    srcs = [
        "radar_process_subnode.cc",
    ],
    hdrs = [
        "object_shared_data.h",
        "radar_process_subnode.h",
    ],
    deps = [
        ":hdmapinput",
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/perception/common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/radar/dummy",
        "//modules/perception/obstacle/radar/modest:modest_detector",
        "//modules/perception/onboard:perception_onboard",
        "@eigen",
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
        ":hdmapinput",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/fusion/probabilistic_fusion",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/onboard:perception_onboard",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "motion_manager_subnode",
    srcs = [
        "motion_manager_service.cc",
    ],
    hdrs = [
        "motion_manager_service.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/camera/motionmanager",
        "//modules/perception/onboard:perception_onboard",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "obstacle_perception",
    srcs = [
        "obstacle_perception.cc",
    ],
    hdrs = [
        "obstacle_perception.h",
        "sensor_raw_frame.h",
    ],
    deps = [
        ":hdmapinput",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/fusion/probabilistic_fusion",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/onboard:lidar_process",
        "//modules/perception/obstacle/radar/modest:modest_detector",
        "//modules/perception/onboard:perception_onboard",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "perception_obstacle_shared_data",
    srcs = [
        "camera_shared_data.cc",
        "fusion_shared_data.cc",
    ],
    hdrs = [
        "camera_shared_data.h",
        "fusion_shared_data.h",
    ],
    deps = [
        "//modules/perception/obstacle/base",
        "//modules/perception/onboard:perception_onboard",
    ],
)

cc_library(
    name = "lane_post_processing_subnode",
    srcs = ["lane_post_processing_subnode.cc"],
    hdrs = [
        "lane_post_processing_subnode.h",
        "lane_shared_data.h",
        "object_shared_data.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/obstacle/base:base",
        "//modules/perception/obstacle/camera/interface:interface",
        "//modules/perception/obstacle/onboard:perception_obstacle_shared_data",
        "//modules/perception/onboard:perception_onboard",
        "//modules/common:log",
        "//modules/perception/common:common",
        "//modules/perception/lib/base:base",
        "@eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "perception_obstacle_lowcost_visualizer_subnode",
    srcs = [
        "visualization_subnode.cc",
    ],
    hdrs = [
        "object_shared_data.h",
        "visualization_subnode.h",
    ],
    deps = [
        ":perception_obstacle_shared_data",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/camera/visualizer",
        "//modules/perception/onboard:perception_onboard",
    ],
)

cc_test(
    name = "hdmap_input_test",
    size = "small",
    srcs = [
        "hdmap_input_test.cc",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/perception:perception_data",
    ],
    deps = [
        ":hdmapinput",
        "//modules/common/configs:config_gflags",
        "@gtest//:main",
    ],
)

cc_test(
    name = "lidar_process_test",
    size = "small",
    srcs = [
        "lidar_process_test.cc",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/perception:perception_data",
    ],
    deps = [
        ":lidar_process",
        "@gtest//:main",
    ],
)

cpplint()
