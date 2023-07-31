load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radar_util",
    srcs = ["radar_util.cc"],
    hdrs = ["radar_util.h"],
    deps = [
        "//modules/perception/common/base",
        "@eigen",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
)

apollo_package()
cpplint()
