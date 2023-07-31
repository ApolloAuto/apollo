load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "semantic_decision",
    srcs = ["semantic_decision.cc"],
    hdrs = ["semantic_decision.h"],
    deps = [
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/camera/common:trafficlight_frame",
        "//modules/perception/traffic_light_tracking/interface:base_traffic_light_tracker",
        "//modules/perception/traffic_light_tracking/tracker/proto:semantic_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
