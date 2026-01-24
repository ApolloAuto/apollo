load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_cc_library(
    name = "lidar_segmention_common",
    srcs = [
        "bg_object_builder.cc",
        "object_split.cc",
        "semantic_builder.cc",
    ],
    hdrs = [
        "bg_object_builder.h",
        "object_split.h",
        "semantic_builder.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "@eigen",
    ],
)

apollo_package(enable_source=False)
cpplint()
