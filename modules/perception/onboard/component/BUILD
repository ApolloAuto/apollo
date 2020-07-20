load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        ":perception_component_inner_detection",
        ":perception_component_inner_fusion",
        ":perception_component_inner_radar",
        ":perception_component_inner_recognition",
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
    name = "perception_component_inner_detection",
    srcs = [
        "detection_component.cc",
    ],
    hdrs = [
        "detection_component.h",
    ],
    deps = [
        ":lidar_inner_component_messages",
        "//cyber",
        "//modules/common/time",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app:lidar_obstacle_detection",
        "//modules/perception/lidar/common",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "@eigen",
    ],
)

cc_library(
    name = "perception_component_inner_fusion",
    srcs = [
        "fusion_component.cc",
    ],
    hdrs = [
        "fusion_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/perception/base",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
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
    name = "perception_component_inner_radar",
    srcs = [
        "radar_detection_component.cc",
    ],
    hdrs = [
        "radar_detection_component.h"
    ],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
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
    name = "perception_component_inner_recognition",
    srcs = [
        "recognition_component.cc",
    ],
    hdrs = [
        "recognition_component.h",
    ],
    deps = [
        ":lidar_inner_component_messages",
        "//cyber",
        "//modules/common/time",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
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
