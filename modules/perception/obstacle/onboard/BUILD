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
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
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
        "//modules/perception/common/sequence_type_fuser",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/lidar/dummy",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "//modules/perception/obstacle/lidar/object_filter/low_object_filter",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/radar/dummy",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "lidar_subnode",
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
        "//modules/perception/common/sequence_type_fuser",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/lidar/dummy",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "//modules/perception/obstacle/lidar/object_filter/low_object_filter",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg",
        "//modules/perception/obstacle/lidar/tracker/hm_tracker",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/onboard",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "radar_subnode",
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
        "//modules/common/time",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/radar/dummy",
        "//modules/perception/obstacle/radar/modest:modest_detector",
        "//modules/perception/onboard",
        "@eigen",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "camera_subnode",
    srcs = [
        "camera_process_subnode.cc",
    ],
    hdrs = [
        "camera_process_subnode.h",
        "object_shared_data.h",
    ],
    deps = [
        ":perception_obstacle_shared_data",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/time",
        "//modules/perception/common",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/camera/converter",
        "//modules/perception/obstacle/camera/detector/yolo_camera_detector",
        "//modules/perception/obstacle/camera/dummy",
        "//modules/perception/obstacle/camera/filter",
        "//modules/perception/obstacle/camera/tracker",
        "//modules/perception/obstacle/camera/transformer",
        "//modules/perception/onboard",
        "//modules/perception/traffic_light/util",
        "@eigen",
        "@opencv2//:core",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "fusion_subnode",
    srcs = [
        "async_fusion_subnode.cc",
        "fusion_subnode.cc",
    ],
    hdrs = [
        "async_fusion_subnode.h",
        "fusion_shared_data.h",
        "fusion_subnode.h",
        "lane_shared_data.h",
        "object_shared_data.h",
    ],
    deps = [
        ":hdmapinput",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/common/time",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/fusion/async_fusion",
        "//modules/perception/obstacle/fusion/probabilistic_fusion",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/onboard",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "motion_subnode",
    srcs = [
        "motion_service.cc",
    ],
    hdrs = [
        "motion_service.h",
    ],
    deps = [
        ":perception_obstacle_shared_data",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/camera/motion",
        "//modules/perception/onboard",
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
        "//modules/perception/obstacle/fusion/async_fusion",
        "//modules/perception/obstacle/fusion/probabilistic_fusion",
        "//modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/onboard:lidar_process",
        "//modules/perception/obstacle/radar/modest:modest_detector",
        "//modules/perception/onboard",
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
        "lane_shared_data.h",
    ],
    deps = [
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
        "//modules/perception/onboard",
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
        ":perception_obstacle_shared_data",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/time",
        "//modules/perception/common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/interface",
        "//modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor",
        "//modules/perception/obstacle/camera/motion",
        "//modules/perception/obstacle/onboard:motion_subnode",
        "@eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "visualization_subnode",
    srcs = [
        "visualization_subnode.cc",
    ],
    hdrs = [
        "object_shared_data.h",
        "visualization_subnode.h",
    ],
    deps = [
        ":perception_obstacle_shared_data",
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/camera/motion",
        "//modules/perception/obstacle/camera/visualizer",
        "//modules/perception/obstacle/onboard:motion_subnode",
        "//modules/perception/onboard",
    ],
)

cc_library(
    name = "cipv_subnode",
    srcs = [
        "cipv_subnode.cc",
    ],
    hdrs = [
        "cipv_subnode.h",
    ],
    deps = [
        ":perception_obstacle_shared_data",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/cipv:camera_cipv",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/camera/motion",
        "//modules/perception/obstacle/camera/visualizer",
        "//modules/perception/obstacle/onboard:lane_post_processing_subnode",
        "//modules/perception/obstacle/onboard:motion_subnode",
        "//modules/perception/onboard",
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
