load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_gap_estimator",
    srcs = ["st_gap_estimator.cc"],
    hdrs = ["st_gap_estimator.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_base/common:planning_gflags",
    ],
)

apollo_package()
cpplint()
