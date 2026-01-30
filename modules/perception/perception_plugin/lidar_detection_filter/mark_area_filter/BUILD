load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libmark_area_filter.so",
    srcs = ["mark_area_filter.cc"],
    hdrs = ["mark_area_filter.h"],
    description = ":plugins.xml",
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/lidar_detection_filter:apollo_perception_lidar_detection_filter",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/perception_plugin/lidar_detection_filter/mark_area_filter/proto:mark_area_filter_proto",
    ],
)

apollo_package(enable_source=False)
cpplint()
