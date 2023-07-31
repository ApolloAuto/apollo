load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

cc_library(
    name = "warm_up",
    srcs = ["warm_up.cc"],
    hdrs = ["warm_up.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/prediction/common:prediction_util",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

apollo_package()
cpplint()
