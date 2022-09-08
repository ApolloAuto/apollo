load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_control_gflags",
    srcs = [
        "sim_control_gflags.cc",
    ],
    hdrs = [
        "sim_control_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "sim_control_util",
    srcs = [
        "sim_control_util.cc",
    ],
    hdrs = [
        "sim_control_util.h",
    ],
    deps = [
        "//cyber",
    ],
)

cpplint()