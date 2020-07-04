load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "yolo_proto",
    deps = [
        ":yolo_proto_lib",
    ],
)

proto_library(
    name = "yolo_proto_lib",
    srcs = ["yolo.proto"],
)

py_proto(
    name = "yolo_pb2",
    src = "yolo.proto",
)
