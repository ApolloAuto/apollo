load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])



cc_binary(
    name = "parser_cli",
    srcs = ["parser_cli.cc"],
    deps = [
        "//modules/drivers/gnss/stream:gnss_stream",
        "//external:gflags",
        "//modules/common",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "test_monitor",
    srcs = ["test_monitor.cc"],
    deps = [
        "//modules/drivers/gnss/stream:gnss_stream",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cpplint()
