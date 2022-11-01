load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "cyber_sysmo_hdrs",
    srcs = glob([
        "*.h",
    ]),
)

cc_test(
    name = "sysmo_test",
    size = "small",
    srcs = ["sysmo_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
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
