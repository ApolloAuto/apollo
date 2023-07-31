load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "poses_interpolation_lib",
    srcs = ["poses_interpolation.cc"],
    hdrs = ["poses_interpolation.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/localization/msf/common/io:common_io",
        "@boost",
    ],
)

apollo_package()
cpplint()
