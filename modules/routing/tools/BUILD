load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "routing_tester",
    srcs = ["routing_tester.cc"],
    deps = [
        "//modules/common",
        "//modules/routing:routing_lib",
        "@ros//:ros_common",
    ],
)

cpplint()
