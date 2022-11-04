load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "all_latest_fusion",
    srcs = ["all_latest_fusion.cc"],
    hdrs = ["all_latest_fusion.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
