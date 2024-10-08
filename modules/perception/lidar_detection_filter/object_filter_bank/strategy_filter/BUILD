load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libstrategy_filter.so",
    description = ":plugins.xml",
    deps = [":lib_bf"],
)

apollo_cc_library(
    name = "lib_bf",
    srcs = ["strategy_filter.cc"],
    hdrs = ["strategy_filter.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/lidar_detection_filter:apollo_perception_lidar_detection_filter",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:strategy_filter_config_cc_proto",
        "//modules/perception/common/lib:apollo_perception_common_lib",
    ],
)

apollo_package()
cpplint()
