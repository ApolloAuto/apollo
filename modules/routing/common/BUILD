load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":routing_gflags",
        ":routing_macros",
        ":routing_util",
    ],
)

cc_library(
    name = "routing_gflags",
    srcs = [
        "routing_gflags.cc",
    ],
    hdrs = [
        "routing_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "routing_util",
    srcs = [
        "utils.cc",
    ],
    hdrs = [
        "utils.h",
    ],
    deps = [
        "//modules/common",
    ],
)

cc_library(
    name = "routing_macros",
    hdrs = [
        "routing_macros.h",
    ],
)

cpplint()
