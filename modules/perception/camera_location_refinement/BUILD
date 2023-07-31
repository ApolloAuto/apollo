load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_location_refinement_files",
    srcs = glob([
        "conf/**",
        "dag/**",
    ])
)

cc_binary(
    name = "libcamera_location_refinement_component_camera.so",
    linkshared = True,
    linkstatic = True,
    deps = [":camera_location_refinement_component"],
)

cc_library(
    name = "camera_location_refinement_component",
    srcs = [
        "camera_location_refinement_component.cc",
    ],
    hdrs = [
        "camera_location_refinement_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/camera_location_refinement/interface:base_postprocessor",
        "//modules/perception/camera_location_refinement/location_refiner:location_refiner_postprocessor",
        "//modules/perception/camera_location_refinement/proto:camera_location_refinement_cc_proto",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:util",
        "//modules/perception/common/lib/calibration_service/online_calibration_service",
        "//modules/perception/common/lib/interface:base_calibration_service",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
    ]
)

apollo_package()
cpplint()
