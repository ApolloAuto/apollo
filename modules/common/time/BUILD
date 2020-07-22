load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "time_util",
    hdrs = ["time_util.h"],
    deps = [
        "//cyber/common:macros",
    ],
)

cc_library(
    name = "time",
    hdrs = ["time.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/common:macros",
        "//cyber/time",
        "//modules/common/configs:config_gflags",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "timer",
    srcs = ["timer.cc"],
    hdrs = [
        "timer.h",
    ],
    deps = [
        ":time",
        ":time_util",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "time_test",
    size = "small",
    srcs = ["time_test.cc"],
    deps = [
        ":time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "timer_test",
    size = "small",
    srcs = ["timer_test.cc"],
    deps = [
        ":timer",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
