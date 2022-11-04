load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "collect_fused_object",
    srcs = ["collect_fused_object.cc"],
    hdrs = ["collect_fused_object.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
