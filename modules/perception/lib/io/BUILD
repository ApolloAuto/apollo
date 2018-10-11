load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "file_util",
    srcs = [
        "file_util.cc",
    ],
    hdrs = [
        "file_util.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//cybertron",
        "//modules/perception/lib/utils:perception_string_util",
    ],
)

cc_test(
    name = "file_util_test",
    size = "small",
    srcs = [
        "file_util_test.cc",
    ],
    data = [
        "//modules/perception:perception_testdata",
    ],
    deps = [
        ":file_util",
        "@gtest//:main",
    ],
)


cc_library(
    name = "protobuf_util",
    hdrs = [
        "protobuf_util.h",
    ],
    deps = [
        "//framework:cybertron",
    ],
)

cpplint()
