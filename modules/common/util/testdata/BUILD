load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "simple_proto",
    deps = [
        ":simple_proto_lib",
    ],
)

proto_library(
    name = "simple_proto_lib",
    srcs = ["simple.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "simple_pb2",
    src = "simple.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)
