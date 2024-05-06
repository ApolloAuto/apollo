load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "motion_service_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ]),
)

apollo_component(
    name = "libmotion_service_component.so",
    srcs = ["motion_service_component.cc",],
    hdrs = ["motion_service_component.h",],
    deps = [
        "//cyber",
        ":apollo_perception_motion_service",
        "//modules/common/math",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "//modules/perception/motion_service/proto:motion_service_component_cc_proto",
    ],
)

apollo_cc_library(
    name = "apollo_perception_motion_service",
    srcs = ["motion/plane_motion.cc"],
    hdrs = ["motion/plane_motion.h"],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/perception/common/base:apollo_perception_common_base",
    ]
)

apollo_package()

cpplint()
