load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_location_refinement_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_camera_location_refinement",
    srcs = [
        "location_refiner/location_refiner_postprocessor.cc",
        "location_refiner/obj_postprocessor.cc",
    ],
    hdrs = [
        "interface/base_postprocessor.h",
        "location_refiner/location_refiner_postprocessor.h",
        "location_refiner/obj_postprocessor.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/camera_location_refinement/location_refiner/proto:location_refiner_cc_proto",
        "//modules/perception/camera_location_refinement/proto:camera_location_refinement_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
    ],
)


apollo_component(
    name = "libcamera_location_refinement_component_camera.so",
    srcs = [
        "camera_location_refinement_component.cc",
    ],
    hdrs = [
        "camera_location_refinement_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        ":apollo_perception_camera_location_refinement",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
    ],
)

apollo_package()

cpplint()
