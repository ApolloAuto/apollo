load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "pointcloud_ground_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_pointcloud_ground_detection",
    srcs = [
        "ground_detector/ground_service_detector/ground_service_detector.cc",
        "ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.cc",
    ],
    hdrs = [
        "ground_detector/ground_service_detector/ground_service_detector.h",
        "ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h",
        "interface/base_ground_detector.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"] + select(
        {
            "@platforms//cpu:x86_64": ["-msse4.1"],
            "@platforms//cpu:aarch64": ["-march=armv8-a+fp+simd+crc"],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/pointcloud_ground_detection/ground_detector/proto:ground_service_detector_config_cc_proto",
        "//modules/perception/pointcloud_ground_detection/ground_detector/proto:spatio_temporal_ground_detector_config_cc_proto",
        "//modules/perception/pointcloud_ground_detection/proto:pointcloud_ground_detection_component_config_cc_proto",
        "@eigen",
    ],
)

apollo_component(
    name = "libpointcloud_ground_detection_component.so",
    srcs = ["pointcloud_ground_detection_component.cc"],
    hdrs = ["pointcloud_ground_detection_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        ":apollo_perception_pointcloud_ground_detection", 
        "//modules/perception/pointcloud_ground_detection/proto:pointcloud_ground_detection_component_config_cc_proto",
    ],
)

# apollo_cc_test(
#     name = "ground_service_detector_test",
#     size = "small",
#     srcs = ["ground_detector/ground_service_detector/ground_service_detector_test.cc"],
#     deps = [
#         ":apollo_perception_pointcloud_ground_detection",
#         "//modules/perception/common:perception_gflags",
#         "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

# apollo_cc_test(
#     name = "spatio_temporal_ground_detector_test",
#     size = "small",
#     srcs = ["ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector_test.cc"],
#     copts = ["-msse4.1"],
#     deps = [
#         ":apollo_perception_pointcloud_ground_detection",
#         "@com_google_googletest//:gtest_main",
#         "@local_config_pcl//:pcl",
#     ],
# )

apollo_package()

cpplint()
