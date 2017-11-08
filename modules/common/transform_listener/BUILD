load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transform_listener",
    hdrs = [
        "transform_listener.h",
    ],
    srcs = [
        "transform_listener.cc",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
        "@ros//:ros_common",
    ],
)

cpplint()
