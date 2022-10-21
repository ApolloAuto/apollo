load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    targets = [
        ":libadapter_gflags.so",
    ],
    deps = ["//modules/common/adapters/proto:install"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libadapter_gflags.so",
    srcs = [
        "adapter_gflags.cc",
        "adapter_gflags.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "adapter_gflags",
    srcs = ["libadapter_gflags.so"],
    hdrs = ["adapter_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True,
    visibility = ["//visibility:public"],
)

cpplint()
