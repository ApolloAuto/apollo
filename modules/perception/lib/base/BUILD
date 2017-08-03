load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "file_util.cc",
        "registerer.cc",
        "timer.cc",
    ],
    hdrs = [
        "file_util.h",
        "registerer.h",
        "singleton.h",
        "timer.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
    ],
)

cc_test(
    name = "perception_lib_base_test",
    size = "small",
    srcs = [
        "file_util_test.cc",
        "registerer_test.cc",
        "singleton_test.cc",
    ],
    deps = [
        ":base",
        "@gtest//:main",
    ],
)

cpplint()
