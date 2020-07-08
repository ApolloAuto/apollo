load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "semantic_decision",
    srcs = ["semantic_decision.cc"],
    hdrs = ["semantic_decision.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/tracker/proto:semantic_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
