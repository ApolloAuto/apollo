load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_location_refinement_files",
    srcs = glob([
        "conf/**",
        "dag/**",
    ])
)

cc_binary(
    name = "libcamera_tracking_component_camera.so",
    linkshared = True,
    linkstatic = True,
    deps = [":camera_tracking_component"],
)

cc_library(
    name = "camera_tracking_component",
    srcs = [
        "camera_tracking_component.cc",
    ],
    hdrs = [
        "camera_tracking_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common:util",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/perception/camera_tracking/base",
        "//modules/perception/camera_tracking/common",
        "//modules/perception/camera_tracking/feature_extract:tracking_feat_extractor",
        "//modules/perception/camera_tracking/interface",
        "//modules/perception/camera_tracking/proto:camera_tracking_component_cc_proto",
        "//modules/perception/camera_tracking/proto:omt_cc_proto",
        "//modules/perception/camera_tracking/proto:tracking_feature_cc_proto",
        "//modules/perception/camera_tracking/tracking:omt_obstacle_tracker",
        "//modules/perception/common/onboard/inner_component_messages",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "//modules/perception/common/algorithm/sensor_manager:sensor_manager",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
