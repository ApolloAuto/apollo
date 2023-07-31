load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_init_options",
    hdrs = ["base_init_options.h"],
    alwayslink = True,
)

cc_library(
    name = "base_calibrator",
    hdrs = ["base_calibrator.h"],
    alwayslink = True,
    deps = [
        ":base_init_options",
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/registerer",
        "@eigen",
    ],
)

cc_library(
    name = "base_calibration_service",
    hdrs = ["base_calibration_service.h"],
    alwayslink = True,
    deps = [
        ":base_init_options",
        "//modules/perception/common/camera/common:camera_frame",
        "//modules/perception/common/lib/registerer:registerer",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
    ],
)

cc_library(
    name = "base_cipv",
    hdrs = ["base_cipv.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        ":base_init_options",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/registerer",
        "@eigen",
    ],
)

apollo_package()
cpplint()
