load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_change_util",
    srcs = ["lane_change_util.cc"],
    hdrs = ["lane_change_util.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:reference_line_info",
    ],
)

apollo_package()
cpplint()
