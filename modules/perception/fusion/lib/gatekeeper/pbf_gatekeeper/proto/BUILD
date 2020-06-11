load("//tools:py_proto.bzl", "py_proto")

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

py_proto(
    name = "pbf_gatekeeper_config_pb2",
    src = "pbf_gatekeeper_config.proto",
)
