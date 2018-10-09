load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = [
        "config_manager.cc",
    ],
    hdrs = [
        "config_manager.h",
    ],
    deps = [
        "//cybertron",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/singleton",
        "//modules/perception/lib/thread",
        "//modules/perception/proto:perception_config_schema_proto",
    ],
)

cc_test(
    name = "config_manager_test",
    size = "small",
    srcs = [
        "config_manager_test.cc",
    ],
    data = [
        "//modules/perception:perception_testdata",
    ],
    deps = [
        ":config_manager",
        "@gtest//:main",
    ],
)

cpplint()
