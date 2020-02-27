load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "examples_cc_proto",
    deps = [
        ":examples_proto",
    ],
)

proto_library(
    name = "examples_proto",
    srcs = ["examples.proto"],
)

py_proto(
    name = "examples_pb2",
    src = "examples.proto",
)
