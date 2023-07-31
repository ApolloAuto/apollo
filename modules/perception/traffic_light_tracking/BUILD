load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "traffic_light_tracking_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libcomponent_traffic_light_tracking.so",
    linkshared = True,
    linkstatic = True,
    deps = [":perception_component_traffic_tracking"],
)

cc_library(
    name = "perception_component_traffic_tracking",
    srcs = [
        "traffic_light_tracking_component.cc",
    ],
    hdrs = [
        "traffic_light_tracking_component.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/perception/common/camera/common:pose",
        "//modules/perception/traffic_light_tracking/interface:base_traffic_light_tracker",
        "//modules/perception/traffic_light_tracking/tracker:semantic_decision",
        "//modules/perception/common/camera/common:trafficlight_frame",
        "//modules/perception/common/onboard/inner_component_messages:traffic_inner_component_messages",
        "//modules/perception/traffic_light_tracking/proto:trafficlights_tracking_component_cc_proto",
        "//modules/perception/common/onboard/common_flags",
        "@opencv//:highgui",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
