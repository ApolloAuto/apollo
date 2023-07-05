load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "dreamview/lib",
    targets = [":libdreamview_gflags.so"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libdreamview_gflags.so",
    srcs = [
        "dreamview_gflags.cc",
        "dreamview_gflags.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "dreamview_gflags",
    srcs = ["libdreamview_gflags.so"],
    hdrs = ["dreamview_gflags.h"],
    alwayslink = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
