load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "map_manager",
    srcs = [
        "map_manager.cc",
    ],
    hdrs = [
        "map_manager.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/common/util:file_util",
        "//modules/perception/base:base_type",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/proto:map_manager_config_proto",
    ],
)

cc_test(
    name = "map_manager_test",
    srcs = [
        "map_manager_test.cc",
    ],
    deps = [
        ":map_manager",
        "@gtest//:main",
    ],
)

cpplint()
