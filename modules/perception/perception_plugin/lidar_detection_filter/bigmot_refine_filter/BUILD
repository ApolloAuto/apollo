load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libbigmot_refine_filter.so",
    description = ":plugins.xml",
    srcs = ["bigmot_refine_filter.cc"],
    hdrs = ["bigmot_refine_filter.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/lidar_detection_filter:apollo_perception_lidar_detection_filter",
        "//modules/perception/perception_plugin/lidar_detection_filter/bigmot_refine_filter/proto:bm_filter_proto",
    ],
)

apollo_package(enable_source=False)
cpplint()
