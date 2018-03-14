load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "empty_predictor",
    srcs = ["empty_predictor.cc"],
    hdrs = ["empty_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/prediction/predictor",
    ],
)

cpplint()
