load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_location_estimation_files",
    srcs = glob([
        "conf/**",
        "dag/**",
    ])
)

cc_binary(
    name = "libcamera_location_estimation_component_camera.so",
    linkshared = True,
    linkstatic = True,
    deps = [":camera_location_estimation_component"],
)

cc_library(
    name = "camera_location_estimation_component",
    srcs = [
        "camera_location_estimation_component.cc",
    ],
    hdrs = [
        "camera_location_estimation_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/camera_location_estimation/interface:base_transformer",
        "//modules/perception/camera_location_estimation/proto:camera_location_estimation_cc_proto",
        "//modules/perception/camera_location_estimation/transformer/multicue:multicue_transformer",
        "//modules/perception/camera_location_estimation/transformer/singlestage:singlestage_transformer",
        "//modules/perception/common/camera/common:util",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
    ]
)

apollo_package()
cpplint()
