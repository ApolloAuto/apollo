load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "traffic_light_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libcomponent_traffic_light_detect.so",
    linkshared = True,
    linkstatic = True,
    deps = [":perception_component_traffic_detect"],
)

cc_library(
    name = "perception_component_traffic_detect",
    srcs = [
        "traffic_light_detection_component.cc",
    ],
    hdrs = [
        "traffic_light_detection_component.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/traffic_light_detection/interface:base_traffic_light_detector",
        "//modules/perception/traffic_light_detection/detection:detection",
        "//modules/perception/common/camera/common:trafficlight_frame",
        "//modules/perception/common/onboard/inner_component_messages:traffic_inner_component_messages",
        "//modules/perception/traffic_light_detection/proto:trafficlights_detection_component_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
