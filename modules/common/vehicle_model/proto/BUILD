load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "vehicle_model_config_proto",
    deps = [
        ":vehicle_model_config_proto_lib",
    ],
)

proto_library(
    name = "vehicle_model_config_proto_lib",
    srcs = ["vehicle_model_config.proto"],
)

py_proto(
    name = "vehicle_model_config_pb2",
    src = "vehicle_model_config.proto",
)
