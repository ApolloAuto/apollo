load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "semantic_decision",
    srcs = ["semantic_decision.cc"],
    hdrs = ["semantic_decision.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:semantic_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
