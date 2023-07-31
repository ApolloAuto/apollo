load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "debug_info",
    srcs = ["debug_info.cc"],
    hdrs = ["debug_info.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/interface:base_calibration_service",
    ],
)

cc_library(
    name = "lane_camera_perception",
    srcs = ["lane_camera_perception.cc"],
    hdrs = ["lane_camera_perception.h"],
    deps = [
        ":debug_info",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base",
        "//modules/perception/lane_detection/proto:perception_cc_proto",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/calibration_service/online_calibration_service",
        "//modules/perception/lane_detection/interface",
        "//modules/perception/common/lib/interface:base_calibration_service",
        "//modules/perception/common/lib/interface:base_calibrator",
        "//modules/perception/lane_detection/lib/detector/darkSCNN:darkSCNN_lane_detector",
        "//modules/perception/lane_detection/lib/detector/denseline:denseline_lane_detector",
        "//modules/perception/lane_detection/lib/postprocessor/darkSCNN:darkSCNN_lane_postprocessor",
        "//modules/perception/lane_detection/lib/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/inference/utils:inference_cuda_util_lib",
        "//modules/perception/common/inference/utils:inference_util_lib",
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
