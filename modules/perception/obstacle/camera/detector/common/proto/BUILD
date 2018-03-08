syntax = "proto2"

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "tracking_feature_proto",
    deps = [
        ":tracking_feature_proto_lib",
    ],
)

proto_library(
    name = "tracking_feature_proto_lib",
    srcs = [
        "tracking_feature.proto",
    ],
)

cpplint()
