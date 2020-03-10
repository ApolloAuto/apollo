load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "sysmo_test",
    size = "small",
    srcs = ["sysmo_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/scheduler:scheduler_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sysmo",
    srcs = ["sysmo.cc"],
    hdrs = ["sysmo.h"],
    deps = [
        "//cyber/scheduler:scheduler_factory",
    ],
)

cpplint()
