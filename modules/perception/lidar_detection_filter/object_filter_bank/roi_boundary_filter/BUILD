load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cyber_plugin_description(
    name = "plugin_libroi_boundary_filter_lib_description",
    description = ":plugins.xml",
    plugin = ":libroi_boundary_filter.so",
)

cc_binary(
    name = "libroi_boundary_filter.so",
    linkshared = True,
    linkstatic = True,
    copts = PERCEPTION_COPTS,
    deps = [":roi_boundary_filter_lib"],
)

cc_library(
    name = "roi_boundary_filter_lib",
    srcs = ["roi_boundary_filter.cc"],
    hdrs = ["roi_boundary_filter.h"],
    deps = [
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/geometry:common",
        "//modules/perception/common/base:point_cloud",
        "//modules/perception/common/lidar/common",
        "//modules/perception/lidar_detection_filter/interface:base_object_filter",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:roi_boundary_filter_config_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "roi_boundary_filter_test",
    size = "small",
    srcs = ["roi_boundary_filter_test.cc"],
    deps = [
        ":roi_boundary_filter_lib",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
