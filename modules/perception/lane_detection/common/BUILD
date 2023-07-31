load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_functions",
    srcs = ["common_functions.cc"],
    hdrs = ["common_functions.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base",
        "@eigen",
    ],
)

apollo_package()
cpplint()
