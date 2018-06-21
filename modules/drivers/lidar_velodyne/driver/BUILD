load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "driver",
    hdrs = [
        "input.h",
        "socket_input.h",
    ],
    deps = [
        "@ros//:ros_common",
    ],
)

cpplint()
