load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decompress_proto",
    deps = [
        ":image_decompress_proto",
    ],
)

cc_proto_library(
    name = "image_decompress_proto",
    deps = [
        ":image_decompress_proto_lib",
    ],
)

proto_library(
    name = "image_decompress_proto_lib",
    srcs = ["config.proto"],
)

py_proto(
    name = "config_pb2",
    src = "config.proto",
)
