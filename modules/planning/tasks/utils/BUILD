load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_gap_estimator",
    srcs = ["st_gap_estimator.cc"],
    hdrs = ["st_gap_estimator.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_gflags",
    ],
)

cpplint()
