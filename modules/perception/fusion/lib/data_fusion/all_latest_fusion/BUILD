load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "all_latest_fusion",
    srcs = ["all_latest_fusion.cc"],
    hdrs = ["all_latest_fusion.h"],
    deps = [
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/pipeline:stage",
        "//modules/common/util:util_tool",
    ],
)