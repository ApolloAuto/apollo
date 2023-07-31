load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_ransac",
    alwayslink = True,
    hdrs = ["i_ransac.h"],
)

apollo_package()
cpplint()
