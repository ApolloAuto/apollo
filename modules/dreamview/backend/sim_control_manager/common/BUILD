load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "dreamview/lib",
    targets = [":libsim_control_gflags.so"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libsim_control_gflags.so",
    srcs = [
        "sim_control_gflags.cc",
        "sim_control_gflags.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "sim_control_gflags",
    srcs = ["libsim_control_gflags.so"],
    hdrs = ["sim_control_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = [
        "interpolation_2d.cc",
    ],
    hdrs = [
        "interpolation_2d.h",
    ],
    alwayslink = True,
)

cc_library(
    name = "sim_control_util",
    srcs = [
        "sim_control_util.cc",
    ],
    hdrs = [
        "sim_control_util.h",
    ],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cpplint()