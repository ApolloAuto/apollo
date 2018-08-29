load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "planning_stats",
    srcs = ["planning_stats.cc"],
    deps = [
        "//modules/common",
        "//modules/planning:planning_base",
        "//modules/planning/proto:planning_stats_proto",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "planning_pad_terminal",
    srcs = ["terminal.cc"],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/planning/proto:planning_proto",
        "@ros//:ros_common",
    ],
)

cpplint()
