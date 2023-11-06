load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "caddn_obstacle_detector",
    srcs = [
        "caddn_obstacle_detector.cc", 
        "postprocess.cc",
    ],
    hdrs = [
        "caddn_obstacle_detector.h", 
        "postprocess.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/interface:base_obstacle_detector",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/camera_detection_single_stage/detector/caddn/proto:model_param_cc_proto",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "@eigen",
        "@opencv//:imgproc",
    ],
)

apollo_package()

cpplint()
