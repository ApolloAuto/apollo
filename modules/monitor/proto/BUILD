load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "system_status_proto",
    deps = [
        ":system_status_proto_lib",
    ],
)

proto_library(
    name = "system_status_proto_lib",
    srcs = ["system_status.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "system_status_pb2",
    src = "system_status.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)
