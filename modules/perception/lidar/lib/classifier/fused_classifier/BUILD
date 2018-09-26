load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        "//modules/perception/base:object",
        "//framework:cybertron",
        "@eigen",
    ],
)

cc_library(
    name = "type_fusion_interface",
    hdrs = [
        "type_fusion_interface.h",
    ],
    deps = [
        "//modules/perception/base:object",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lidar/common:common",
    ],
)

cc_library(
    name = "ccrf_type_fusion",
    srcs = [
        "ccrf_type_fusion.cc",
    ],
    hdrs = [
        "ccrf_type_fusion.h",
    ],
    deps = [
        ":util",
        ":type_fusion_interface",
        "//modules/common/util:file_util",
        "//modules/perception/base:object",
        "//modules/perception/base:point_cloud",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lidar/common:object_sequence",
        "//modules/perception/proto:ccrf_type_fusion_config_proto",
        "//framework:cybertron",
    ],
)

cpplint()
