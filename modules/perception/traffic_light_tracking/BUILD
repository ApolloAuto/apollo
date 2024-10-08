load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"trafficlight\\"']

filegroup(
    name = "traffic_light_tracking_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_traffic_light_tracking",
    srcs = [
        "tracker/semantic_decision.cc",
        "traffic_light_tracking_component.cc",
    ],
    hdrs = [
        "interface/base_traffic_light_tracker.h",
        "tracker/semantic_decision.h",
        "traffic_light_tracking_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_tracking/proto:trafficlights_tracking_component_cc_proto",
        "//modules/perception/traffic_light_tracking/tracker/proto:semantic_cc_proto",
        "@opencv//:highgui",
    ],
)

apollo_component(
    name = "libcomponent_traffic_light_tracking.so",
    srcs = [
        "traffic_light_tracking_component.cc",
    ],
    hdrs = [
        "traffic_light_tracking_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        ":apollo_perception_traffic_light_tracking",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_tracking/proto:trafficlights_tracking_component_cc_proto",
        "@opencv//:highgui",
    ],
)

apollo_package()

cpplint()
