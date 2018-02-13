load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "navigation_proto",
    deps = [
        ":navigation_proto_lib",
    ],
)

proto_library(
    name = "navigation_proto_lib",
    srcs = [
        "navigation.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

cpplint()
