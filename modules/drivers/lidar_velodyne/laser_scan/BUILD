load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "laser_scan",
    srcs = [
        "laser_scan.cc",
    ],
    hdrs = [
        "laser_scan.h",
    ],
    deps = [
        "//framework:cybertron",
        "@ros//:ros_common",
    ],
)

cpplint()
