load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_flags.so",
    srcs = ["common_flags.cc", "common_flags.h"],
    linkstatic = True,
    linkshared = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "common_flags",
    srcs = [":libcommon_flags.so"],
    hdrs = ["common_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

install(
    name = "install",
    library_dest = "perception/lib",
    targets = [
        ":libcommon_flags.so",
    ],
    visibility = ["//visibility:public"],
)

cpplint()
