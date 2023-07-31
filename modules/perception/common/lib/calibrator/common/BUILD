load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "histogram_estimator",
    srcs = ["histogram_estimator.cc"],
    hdrs = ["histogram_estimator.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm/i_lib/core",
        "@eigen",
    ],
)

apollo_package()
cpplint()
