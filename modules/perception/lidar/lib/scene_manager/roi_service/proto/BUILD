load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "roi_service_proto",
    deps = [
        ":roi_service_proto_lib",
    ],
)

proto_library(
    name = "roi_service_proto_lib",
    srcs = ["roi_service.proto"],
)

py_proto(
    name = "roi_service_pb2",
    src = "roi_service.proto",
    with_grpc = True,
)
