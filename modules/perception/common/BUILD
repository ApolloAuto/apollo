load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
        "file_system_util.cc",
        "graph_util.cc",
        "perception_gflags.cc",
    ],
    hdrs = [
        "file_system_util.h",
        "graph_util.h",
        "perception_gflags.h",
    ],
    deps = [
        "//modules/common",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "file_system_util_test.cc",
        "graph_util_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
    ],
    deps = [
        ":common",
        "@eigen",
        "@gtest//:main",
    ],
)

cpplint()
