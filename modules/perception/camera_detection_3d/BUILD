load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_detection_3d_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ]),
)

apollo_component(
    name = "libcamera_detection_3d_component_camera.so",
    srcs = [
        "camera_detection_3d_component.cc",
    ],
    hdrs = [
        "camera_detection_3d_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_3d/detector/caddn:caddn_obstacle_detector",
        "//modules/perception/camera_detection_3d/detector/smoke:smoke_obstacle_detector",
        "//modules/perception/camera_detection_3d/proto:camera_detection_3d_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
    ],
)

apollo_package()

cpplint()
