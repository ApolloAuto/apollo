load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "pbf_gatekeeper_config_proto",
    deps = [
        ":pbf_gatekeeper_config_proto_lib",
    ],
)

proto_library(
    name = "pbf_gatekeeper_config_proto_lib",
    srcs = ["pbf_gatekeeper_config.proto"],
)

cpplint()
