load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "task",
    hdrs = ["task.h"],
    deps = [
        ":task_manager",
    ],
)

cc_test(
    name = "task_test",
    size = "small",
    srcs = ["task_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "task_manager",
    srcs = ["task_manager.cc"],
    hdrs = ["task_manager.h"],
    copts = ["-faligned-new"],
    deps = [
        "//cyber/scheduler:scheduler_factory",
    ],
)

cpplint()
