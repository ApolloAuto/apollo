load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_gflags",
    srcs = [
        "velodyne_gflags.cc",
    ],
    hdrs = [
        "velodyne_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)


cc_library(
    name = "common",
    srcs = [
        "input.cc",
        "socket_input.cc",
    ],
    hdrs = [
        "input.h",
        "socket_input.h",
    ],
    deps = [
        "//modules/common",
        "@ros//:ros_common",
    ],
)

cpplint()
