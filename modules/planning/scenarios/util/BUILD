load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/planning/common:planning_common",
    ],
)

cpplint()
