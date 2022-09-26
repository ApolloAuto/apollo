load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "perception/lib/camera/lib/motion_service",
    targets = [":libmotion_service.so"],
)

cc_library(
    name = "motion_service_lib",
    srcs = ["motion_service.cc"],
    hdrs = ["motion_service.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface:base_init_options",
        "//modules/perception/camera/lib/motion",
        "//modules/perception/lib/registerer",
        "//modules/perception/onboard/proto:motion_service_cc_proto",
        "//modules/perception/proto:motion_service_cc_proto",
    ],
)

cc_binary(
    name = "libmotion_service.so",
    linkshared = True,
    linkstatic = True,
    deps = [":motion_service_lib"],
)

cpplint()
