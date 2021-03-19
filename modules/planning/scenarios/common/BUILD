load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stage_intersection_cruise_impl",
    srcs = ["stage_intersection_cruise_impl.cc"],
    hdrs = ["stage_intersection_cruise_impl.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios/util:scenario_util_lib",
    ],
)

cpplint()
