load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "ncut_param_proto_lib",
    srcs = [
        "ncut_config.proto",
        "ncut_param.proto",
    ],
)

cc_proto_library(
    name = "ncut_param_proto",
    deps = [
        ":ncut_param_proto_lib",
    ],
)

py_proto(
    name = "ncut_config_pb2",
    src = "ncut_config.proto",
)

py_proto(
    name = "ncut_param_pb2",
    src = "ncut_param.proto",
)
