load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transform_wrapper",
    srcs = ["transform_wrapper.cc"],
    hdrs = ["transform_wrapper.h"],
    deps = [
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cpplint()
