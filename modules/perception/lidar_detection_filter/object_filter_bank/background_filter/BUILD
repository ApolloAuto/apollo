load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cyber_plugin_description(
    name = "plugin_libbackground_filter_lib_description",
    description = ":plugins.xml",
    plugin = ":libbackground_filter.so",
)

cc_binary(
    name = "libbackground_filter.so",
    linkshared = True,
    linkstatic = True,
    copts = PERCEPTION_COPTS,
    deps = [":background_filter_lib"],
)

cc_library(
    name = "background_filter_lib",
    srcs = ["background_filter.cc"],
    hdrs = ["background_filter.h"],
    deps = [
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/geometry:roi_filter",
        "//modules/perception/common/algorithm/geometry:common",
        "//modules/perception/lidar_detection_filter/interface:base_object_filter",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:background_filter_config_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
