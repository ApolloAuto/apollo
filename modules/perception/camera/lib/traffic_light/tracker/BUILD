load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "trafficlight_tracker_semantic_proto",
    deps = [
        ":trafficlight_tracker_semantic_lib",
    ],
)

proto_library(
    name = "trafficlight_tracker_semantic_lib",
    srcs = ["semantic.proto"],
)

cc_library(
    name = "semantic_decision",
    srcs = ["semantic_decision.cc"],
    hdrs = ["semantic_decision.h"],
    deps = [
        ":trafficlight_tracker_semantic_proto",
        "//cyber/common:file",
        "//modules/perception/camera/lib/interface",
    ],
    alwayslink = True,
)

cpplint()
