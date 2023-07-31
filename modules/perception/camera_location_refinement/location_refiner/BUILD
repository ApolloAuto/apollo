load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "location_refiner_postprocessor",
    srcs = ["location_refiner_postprocessor.cc"],
    hdrs = ["location_refiner_postprocessor.h"],
    deps = [
        ":obj_postprocessor",
        "//cyber",
        "//modules/perception/camera_location_refinement/interface:base_postprocessor",
        "//modules/perception/camera_location_refinement/location_refiner/proto:location_refiner_cc_proto",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/camera/common:global_config",
        "//modules/perception/common/lib/interface:base_calibration_service",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
    ],
    alwayslink = True,
)

cc_library(
    name = "obj_postprocessor",
    srcs = ["obj_postprocessor.cc"],
    hdrs = ["obj_postprocessor.h"],
    deps = [
        "//modules/perception/common/camera/common:twod_threed_util",
    ],
)

apollo_package()
cpplint()
