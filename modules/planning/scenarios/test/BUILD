load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stage_test_base",
    srcs = [
        "stage_test_base.cc",
    ],
    hdrs = [
        "stage_test_base.h",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/planning/tasks:task_factory",
        "@gtest",
    ],
)

cpplint()
