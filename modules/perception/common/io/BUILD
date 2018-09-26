load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "io_util",
    srcs = [
        "io_util.cc",
    ],
    hdrs = [
        "io_util.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/perception/base:camera",
        "//modules/perception/base:distortion_model",
        "//modules/perception/base:omnidirectional_model",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/lib/io:file_util",
    ],
)

cc_test(
    name = "io_util_test",
    size = "small",
    srcs = [
        "io_util_test.cc",
    ],
    data = [
        "//modules/perception:perception_testdata",
    ],
    deps = [
        ":io_util",
        "@yaml_cpp//:yaml",
        "@gtest//:main",
    ],
)

cpplint()
