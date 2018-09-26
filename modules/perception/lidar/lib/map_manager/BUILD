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
        "//modules/common/util:file_util",
        "//modules/perception/proto:map_manager_config_proto",
        "//modules/perception/base:base_type",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//framework:cybertron",
    ],
)

cpplint()
