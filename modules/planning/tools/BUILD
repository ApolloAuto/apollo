load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "planning_stats",
    srcs = ["planning_stats.cc"],
    deps = [
        "//modules/planning:planning_lib",
        "//modules/common",
        "@ros//:ros_common",
    ],
)

cpplint()
