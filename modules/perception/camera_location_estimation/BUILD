load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_location_estimation_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_camera_location_estimation",
    srcs = [
        "transformer/multicue/multicue_transformer.cc",
        "transformer/multicue/obj_mapper.cc",
        "transformer/singlestage/singlestage_transformer.cc",
    ],
    hdrs = [
        "interface/base_transformer.h",
        "transformer/multicue/multicue_transformer.h",
        "transformer/multicue/obj_mapper.h",
        "transformer/singlestage/singlestage_transformer.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/camera_location_estimation/proto:camera_location_estimation_cc_proto",
        "//modules/perception/camera_location_estimation/transformer/multicue/proto:multicue_cc_proto",
        "//modules/perception/camera_location_estimation/transformer/singlestage/proto:singlestage_cc_proto",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "@eigen",
    ],
)


apollo_component(
    name = "libcamera_location_estimation_component_camera.so",
    srcs = [
        "camera_location_estimation_component.cc",
    ],
    hdrs = [
        "camera_location_estimation_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "apollo_perception_camera_location_estimation",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
    ],
)

apollo_package()

cpplint()
