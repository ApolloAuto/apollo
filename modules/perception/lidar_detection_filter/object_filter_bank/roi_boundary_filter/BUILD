load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libroi_boundary_filter.so",
    srcs = ["roi_boundary_filter.cc"],
    hdrs = ["roi_boundary_filter.h"],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/lidar_detection_filter:apollo_perception_lidar_detection_filter",
        "//modules/perception/lidar_detection_filter/object_filter_bank/proto:roi_boundary_filter_config_cc_proto",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
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
