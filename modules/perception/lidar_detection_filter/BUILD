load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_detection_filter_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_lidar_detection_filter",
    srcs = [
        "object_filter_bank/object_filter_bank.cc",
    ],
    hdrs = [
        "interface/base_object_filter.h",
        "object_filter_bank/object_filter_bank.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
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
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:background_filter_config_cc_proto",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:strategy_filter_config_cc_proto",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:filter_bank_config_cc_proto",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:roi_boundary_filter_config_cc_proto",
        "//modules/perception/lidar_detection_filter/proto:lidar_detection_filter_component_config_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_component(
    name = "liblidar_detection_filter_component.so",
    srcs = ["lidar_detection_filter_component.cc"],
    hdrs = ["lidar_detection_filter_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        ":apollo_perception_lidar_detection_filter",
        "//modules/perception/lidar_detection_filter/proto:lidar_detection_filter_component_config_cc_proto",
    ],
)

apollo_package()

cpplint()
