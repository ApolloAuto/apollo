load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_sort",
    hdrs = ["i_sort.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/algorithm/i_lib/core",
    ],
)

apollo_package()
cpplint()
