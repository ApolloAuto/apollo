load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "traffic_light_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_traffic_light_detection",
    srcs = [
        "detection/cropbox.cc",
        "detection/detection.cc",
        "detection/select.cc",
        "traffic_light_detection_component.cc",
    ],
    hdrs = [
        "detection/cropbox.h",
        "detection/detection.h",
        "detection/select.h",
        "interface/base_traffic_light_detector.h",
        "traffic_light_detection_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_detection/detection/proto:model_param_cc_proto",
        "//modules/perception/traffic_light_detection/proto:trafficlights_detection_component_cc_proto",
    ],
)

apollo_component(
    name = "libcomponent_traffic_light_detect.so",
    srcs = [
        "traffic_light_detection_component.cc",
    ],
    hdrs = [
        "traffic_light_detection_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        ":apollo_perception_traffic_light_detection",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_detection/proto:trafficlights_detection_component_cc_proto",
    ],
)

apollo_package()

cpplint()
