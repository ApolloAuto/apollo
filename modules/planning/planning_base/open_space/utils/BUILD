load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "open_space_roi_utils",
    srcs = ["open_space_roi_util.cc"],
    hdrs = ["open_space_roi_util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/planning_base/common:frame"
    ],
)

apollo_package()
cpplint()
