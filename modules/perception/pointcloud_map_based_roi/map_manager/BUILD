load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "map_manager",
    srcs = ["map_manager.cc"],
    hdrs = ["map_manager.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/map/hdmap",
        "//modules/perception/common:util",
        "//modules/perception/common/base:base_type",
        "//modules/perception/common/lidar/common:lidar_frame",
        "//modules/perception/common/hdmap:hdmap_input",
        "//modules/perception/pointcloud_map_based_roi/map_manager/proto:map_manager_config_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
        "@com_google_googletest//:gtest",
    ],
)

#cc_test(
#    name = "map_manager_test",
#    srcs = ["map_manager_test.cc"],
#    deps = [
#        ":map_manager",
#        "@com_google_googletest//:gtest_main",
#    ],
#)

apollo_package()
cpplint()
