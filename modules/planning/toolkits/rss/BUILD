load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rss_task",
    srcs = [
        "rss_task.cc",
    ],
    hdrs = [
        "rss_task.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//third_party/rss:rss",
        "//modules/common/status",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/toolkits:task",
    ],
)

cc_test(
    name = "rss_task_test",
    size = "small",
    srcs = [
        "rss_task_test.cc",
    ],
    deps = [
        ":rss_task",
        "//third_party/rss:rss",
        "@gtest//:main",
    ],
)

cpplint()
