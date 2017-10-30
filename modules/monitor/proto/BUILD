load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "hardware_status_proto",
    deps = [
        ":hardware_status_proto_lib",
    ],
)

proto_library(
    name = "hardware_status_proto_lib",
    srcs = ["hardware_status.proto"],
)

cpplint()
