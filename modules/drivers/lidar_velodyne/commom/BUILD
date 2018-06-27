load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
        "input.cc",
    ],
    hdrs = [
        "input.h",
    ],
    deps = [
        "//modules/common",
        "@ros//:ros_common",
    ],
)

cpplint()
