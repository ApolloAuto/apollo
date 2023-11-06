load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "offline_camera_detection",
    srcs = ["offline_camera_detection.cc"],
    deps = [
        "//cyber",
        "//modules/perception/camera_detection_multi_stage:apollo_perception_camera_detection_multi_stage",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/interface:base_obstacle_detector",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/tools/common:apollo_perception_tools_common",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()
