load("//tools:py_proto.bzl", "py_proto")
load("@com_github_grpc_grpc//bazel:cc_grpc_library.bzl", "cc_grpc_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "map_datachecker_proto",
    deps = [
        ":map_datachecker_proto_lib",
    ],
)

proto_library(
    name = "map_datachecker_proto_lib",
    srcs = [
        "collection_check_message.proto",
        "collection_error_code.proto",
    ],
)

cc_grpc_library(
    name = "map_datachecker_grpc",
    srcs = ["collection_service.proto"],
    deps = [
        ":map_datachecker_proto_lib",
    ],
)

py_proto(
    name = "collection_error_code_pb2",
    src = "collection_error_code.proto",
)

py_proto(
    name = "collection_check_message_pb2",
    src = "collection_check_message.proto",
    deps = [
        "//modules/map/tools/map_datachecker/proto:collection_error_code_pb2",
    ],
)

py_proto(
    name = "collection_service_pb2",
    src = "collection_service.proto",
    with_grpc = True,
    deps = [
        "//modules/map/tools/map_datachecker/proto:collection_check_message_pb2",
    ],
)
