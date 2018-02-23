load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "registerer.cc",
        "thread.cc",
        "timer.cc",
    ],
    hdrs = [
        "concurrent_queue.h",
        "mutex.h",
        "registerer.h",
        "thread.h",
        "time_util.h",
        "timer.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "@gtest",
    ],
)

cc_test(
    name = "perception_lib_base_test",
    size = "small",
    srcs = [
        "registerer_test.cc",
        "timer_test.cc",
    ],
    data = ["//modules/perception:perception_data"],
    deps = [
        ":base",
        "@gtest//:main",
    ],
)

cpplint()
