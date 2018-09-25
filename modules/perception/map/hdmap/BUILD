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
    ],
)

cc_test(
    name = "hdmap_input_test",
    size = "small",
    srcs = [
        "hdmap_input_test.cc",
    ],
    deps = [
        ":hdmap_input",
        "@gtest//:main",
    ],
)

cpplint()
