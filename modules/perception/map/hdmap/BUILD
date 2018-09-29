load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hdmap_input",
    srcs = [
        "hdmap_input.cc",
    ],
    hdrs = [
        "hdmap_input.h",
    ],
    deps = [
        "//modules/perception/base:base_type",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/geometry:common",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/singleton",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/map/proto:map_proto",
        "//modules/map/hdmap:hdmap",
        "//modules/common/proto:geometry_proto",
        "//modules/common/math:geometry",
        "//modules/perception/base:common",
        "//modules/perception/base:object_pool",
        "//modules/perception/base:frame",
        "//modules/perception/base:blob",
        "//modules/perception/base:syncedmem",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/base:image",
        "//modules/perception/base:impending_collision_edge",
        "//modules/perception/base:object",
    ],
)

#cc_test(
#    name = "hdmap_input_test",
#    size = "small",
#    srcs = [
#        "hdmap_input_test.cc",
#    ],
#    deps = [
#        ":hdmap_input",
#        "@gtest//:main",
#        "//modules/perception/base:base_type",
#        "//modules/perception/base:point_cloud",
#        "//modules/perception/common/geometry:common",
#        "//modules/perception/lib/config_manager",
#        "//modules/perception/lib/singleton",
#        "//modules/perception/proto:perception_config_schema_proto",
#    ],
#)

cpplint()
