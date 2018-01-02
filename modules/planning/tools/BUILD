load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "planning_stats",
    srcs = ["planning_stats.cc"],
    deps = [
        "//modules/common",
        "//modules/planning:planning_lib",
        "//modules/planning/proto:planning_stats_proto",
        "@ros//:ros_common",
    ],
)

cpplint()
