load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

install(
    name = "install",
    library_dest = "canbus/lib",
    targets = [":libcanbus_gflags.so"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libcanbus_gflags.so",
    srcs = [
        "canbus_gflags.cc",
        "canbus_gflags.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "canbus_gflags",
    srcs = ["libcanbus_gflags.so"],
    hdrs = ["canbus_gflags.h"],
    alwayslink = True,
    copts = CANBUS_COPTS,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
