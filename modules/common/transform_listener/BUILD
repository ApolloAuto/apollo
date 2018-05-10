load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transform_listener",
    srcs = [
        "transform_listener.cc",
    ],
    hdrs = [
        "transform_listener.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
        "@ros//:ros_common",
    ],
)

cpplint()
