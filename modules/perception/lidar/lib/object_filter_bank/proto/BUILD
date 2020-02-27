load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "filter_bank_config_proto",
    deps = [
        ":filter_bank_config_proto_lib",
    ],
)

proto_library(
    name = "filter_bank_config_proto_lib",
    srcs = ["filter_bank_config.proto"],
)

py_proto(
    name = "filter_bank_config_pb2",
    src = "filter_bank_config.proto",
)
