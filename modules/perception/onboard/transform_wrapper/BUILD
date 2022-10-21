load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libtransform_wrapper.so",
    srcs = ["transform_wrapper.cc", "transform_wrapper.h"],
    linkstatic = True,
    linkshared = True,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_library(
    name = "transform_wrapper",
    hdrs = ["transform_wrapper.h"],
    srcs = [":libtransform_wrapper.so"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen"
    ]
)

install(
    name = "install",
    library_dest = "perception/lib",
    targets = [
        ":libtransform_wrapper.so",
    ],
    visibility = ["//visibility:public"],
)

cpplint()

