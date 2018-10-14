load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "task_lib",
    srcs = glob([
        "task_manager.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/scheduler:scheduler_lib",
    ],
)

cc_test(
    name = "task_test",
    size = "small",
    srcs = [
        "task_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
