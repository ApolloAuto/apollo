load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cyber_plugin_description(
    name = "plugin_libroi_service_filter_lib_description",
    description = ":plugins.xml",
    plugin = ":libroi_service_filter.so",
)

cc_binary(
    name = "libroi_service_filter.so",
    linkshared = True,
    linkstatic = True,
    copts = PERCEPTION_COPTS,
    deps = [":roi_service_filter_lib"],
)

cc_library(
    name = "roi_service_filter_lib",
    srcs = ["roi_service_filter.cc"],
    hdrs = ["roi_service_filter.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/lidar/common:lidar_frame",
        "//modules/perception/common/lidar/common:lidar_log",
        "//modules/perception/common/lidar/common:lidar_point_label",
        "//modules/perception/common/lidar/scene_manager",
        "//modules/perception/common/lidar/scene_manager/roi_service",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/pointcloud_map_based_roi/interface:base_roi_filter",
        "//modules/perception/common/lib/interface:base_init_options",
        "@eigen",
    ],
)

#cc_test(
#    name = "roi_service_filter_test",
#    size = "small",
#    srcs = ["roi_service_filter_test.cc"],
#    deps = [
#        ":roi_service_filter",
#        "//modules/perception/common:perception_gflags",
#        "//modules/perception/common/io:io_util",
#        "//modules/perception/lidar/common:pcl_util",
#        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
#        "//modules/perception/map/hdmap:hdmap_input",
#        "//modules/perception/lidar/lib/scene_manager/ground_service:ground_service",
#        "@com_google_googletest//:gtest_main",
#    ],
#)

apollo_package()
cpplint()
