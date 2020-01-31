load("//tools:cpplint.bzl", "cpplint")
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

cpplint()
