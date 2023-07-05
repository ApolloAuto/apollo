load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_binary(
    name = "libperception_component_camera.so",
    linkshared = True,
    linkstatic = True,
    deps = [":perception_component_inner_camera"],
)

install(
    name = "install",
    library_dest = "perception/lib/onboard/component",
    targets = [
        ":libperception_component_camera.so",
        ":libperception_component_lidar.so",
    ],
    deps = [
        "//modules/perception/production:install",
    ],
)

cc_library(
    name = "perception_component_inner_camera",
    srcs = [
        # todo(zero): need fix
        "camera_obstacle_detection_component.cc",
        "camera_perception_viz_message.cc",
        "fusion_camera_detection_component.cc",
        "lane_detection_component.cc",
        "trafficlights_perception_component.cc",
        "camera_bev_detection_component.cc"
    ],
    hdrs = [
        "camera_obstacle_detection_component.h",
        "camera_perception_viz_message.h",
        "fusion_camera_detection_component.h",
        "lane_detection_component.h",
        "trafficlights_perception_component.h",
        "camera_bev_detection_component.h"
    ],
    linkopts = ["-lleveldb"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/map_msgs:map_geometry_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_camera_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_lane_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/camera/app:cipv_camera",
        "//modules/perception/camera/app:lane_camera_perception",
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/camera/app:obstacle_detection_camera",
        "//modules/perception/camera/app:traffic_light_camera_perception",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/motion_service:motion_service_lib",
        "//modules/perception/camera/lib/traffic_light/preprocessor:tl_preprocessor",
        "//modules/perception/camera/tools/offline:transform_server",
        "//modules/perception/camera/tools/offline:visualizer",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:fusion_camera_detection_component_cc_proto",
        "//modules/perception/onboard/proto:lane_perception_component_cc_proto",
        "//modules/perception/onboard/proto:trafficlights_perception_component_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/pipeline/proto:traffic_light_config_cc_proto",
        "@eigen",
        "@opencv//:highgui",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libperception_component_lidar.so",
    linkshared = True,
    linkstatic = True,
    deps = [":perception_component_inner_lidar"],
)

cc_library(
    name = "perception_component_inner_lidar",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    linkopts = ["-lleveldb"],
    deps = [
        ":lidar_detection_component",
        ":multi_sensor_fusion_component",
        ":radar_detection_component",
        ":lidar_tracking_component",
        ":lidar_output_component",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_detection_component",
    srcs = ["lidar_detection_component.cc"],
    hdrs = ["lidar_detection_component.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detection",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/detector/cnn_segmentation:cnn_segmentation",
        "//modules/perception/lidar/lib/detector/ncut_segmentation:ncut_segmentation",
        "//modules/perception/lidar/lib/detector/point_pillars_detection:point_pillars_detection",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "multi_sensor_fusion_component",
    srcs = ["multi_sensor_fusion_component.cc"],
    hdrs = ["multi_sensor_fusion_component.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        # "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/fusion/lib/interface:base_multisensor_fusion",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:fusion_component_config_cc_proto",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "radar_detection_component",
    srcs = [
        "radar_detection_component.cc",
    ],
    hdrs = [
        "radar_detection_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_buffer",
        "//modules/perception/onboard/proto:radar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/radar/app:radar_obstacle_perception",
        "//modules/perception/radar/common:radar_util",
        "//modules/perception/radar/lib/detector/conti_ars_detector",
        "//modules/perception/radar/lib/dummy:dummy_algorithms",
        "//modules/perception/radar/lib/interface:base_detector",
        "//modules/perception/radar/lib/interface:base_filter",
        "//modules/perception/radar/lib/interface:base_matcher",
        "//modules/perception/radar/lib/interface:base_preprocessor",
        "//modules/perception/radar/lib/interface:base_radar_obstacle_perception",
        "//modules/perception/radar/lib/interface:base_roi_filter",
        "//modules/perception/radar/lib/interface:base_tracker",
        "//modules/perception/radar/lib/preprocessor/conti_ars_preprocessor",
        "//modules/perception/radar/lib/roi_filter/hdmap_radar_roi_filter",
        "//modules/perception/radar/lib/tracker/common:radar_track",
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "//modules/perception/radar/lib/tracker/conti_ars_tracker",
        "//modules/perception/radar/lib/tracker/filter:adaptive_kalman_filter",
        "//modules/perception/radar/lib/tracker/matcher:hm_matcher",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_tracking_component",
    srcs = [
        "lidar_tracking_component.cc",
    ],
    hdrs = [
        "lidar_tracking_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_tracking",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_output_component",
    srcs = [
        "lidar_output_component.cc",
    ],
    hdrs = [
        "lidar_output_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/onboard/msg_serializer",
    ],
    alwayslink = True,
)

cpplint()
