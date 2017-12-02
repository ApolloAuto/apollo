load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "file_util.cc",
        "registerer.cc",
        "timer.cc",
        "thread.cc",
    ],
    hdrs = [
        "file_util.h",
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
        "file_util_test.cc",
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
