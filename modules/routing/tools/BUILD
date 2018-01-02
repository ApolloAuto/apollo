load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "routing_tester",
    srcs = ["routing_tester.cc"],
    deps = [
        "//modules/routing:routing_lib",
        "//modules/common",
        "@ros//:ros_common",
    ],
)

cpplint()
