load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "motion_service_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libmotion_service_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":motion_service_component_lib"],
)

cc_library(
    name = "motion_service_component_lib",
    srcs = ["motion_service_component.cc"],
    hdrs = ["motion_service_component.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common/math",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "//modules/perception/motion_service/motion:plane_motion",
        "//modules/perception/motion_service/proto:motion_service_component_cc_proto",
    ],
)

apollo_package()
cpplint()
