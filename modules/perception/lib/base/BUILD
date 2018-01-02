load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "registerer.cc",
        "timer.cc",
        "thread.cc",
    ],
    hdrs = [
        "registerer.h",
        "timer.h",
        "time_util.h",
        "mutex.h",
        "thread.h",
        "concurrent_queue.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "@gtest//:gtest",
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
