load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PERCEPTION_COPTS = [ '-DMODULE_NAME=\\"perception\\"' ]

cc_binary(
    name = "libperception_component_camera.so",
    linkshared = True,
    linkstatic = False,
    deps = [":perception_component_inner_camera"],
)

cc_library(
    name = "perception_component_inner_camera",
    srcs = [
        "trafficlights_perception_component.cc",
    ],
    hdrs = [
        "trafficlights_perception_component.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util:time_util",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/map/proto:map_cc_proto",
        "//modules/perception/base",
        "//modules/perception/camera/app:traffic_light_camera_perception",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/traffic_light/preprocessor:tl_preprocessor",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/proto:trafficlights_perception_component_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_traffic_light_cc_proto",
        "@eigen",
        "@opencv//:highgui",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libperception_component_lidar.so",
    linkshared = True,
    linkstatic = False,
    deps = [":perception_component_inner_lidar"],
)

cc_library(
    name = "perception_component_inner_lidar",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":detection_component",
        ":fusion_component",
        ":radar_detection_component",
        ":recognition_component",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_inner_component_messages",
    hdrs = [
        "lidar_inner_component_messages.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/lidar/common",
        "//modules/perception/onboard/inner_component_messages",
    ],
)

cc_library(
    name = "detection_component",
    srcs = ["detection_component.cc"],
    hdrs = ["detection_component.h"],
    deps = [
        ":lidar_inner_component_messages",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detection",
        "//modules/perception/lidar/common",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "@eigen",
    ],
)

cc_library(
    name = "fusion_component",
    srcs = ["fusion_component.cc"],
    hdrs = ["fusion_component.h"],
    deps = [
        "//cyber/time:clock",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:fusion_component_config_cc_proto",
        "@eigen",
    ],
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
        "//cyber/time:clock",
        "//modules/common/util:perf_util",
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
)

cc_library(
    name = "recognition_component",
    srcs = [
        "recognition_component.cc",
    ],
    hdrs = [
        "recognition_component.h",
    ],
    deps = [
        ":lidar_inner_component_messages",
        "//cyber/time:clock",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_tracking",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
    ],
)

cpplint()
