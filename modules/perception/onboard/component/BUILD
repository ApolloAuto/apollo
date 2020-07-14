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
    srcs = [
        "detection_component.cc",
        "fusion_component.cc",
        "recognition_component.cc",
    ],
    hdrs = [
        "detection_component.h",
        "fusion_component.h",
        "lidar_inner_component_messages.h",
        "recognition_component.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_buffer",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/proto:fusion_component_config_cc_proto",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/transform:tf2_buffer_lib",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
