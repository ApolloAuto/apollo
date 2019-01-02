load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider_rss",
    srcs = [
        "decider_rss.cc",
    ],
    hdrs = [
        "decider_rss.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:task",
        "//third_party/rss",
    ],
)

cc_test(
    name = "decider_rss_test",
    size = "small",
    srcs = [
        "decider_rss_test.cc",
    ],
    deps = [
        ":decider_rss",
        "//third_party/rss",
        "@gtest//:main",
    ],
)

cpplint()
