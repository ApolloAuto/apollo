load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motion_service_lib",
    srcs = ["motion_service.cc"],
    hdrs = ["motion_service.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
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
    linkstatic = False,
    deps = [":motion_service_lib"],
)

cpplint()
