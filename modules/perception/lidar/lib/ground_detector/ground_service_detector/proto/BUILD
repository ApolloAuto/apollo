load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "ground_service_detector_config_proto",
    deps = [
        ":ground_service_detector_config_proto_lib",
    ],
)

proto_library(
    name = "ground_service_detector_config_proto_lib",
    srcs = ["ground_service_detector_config.proto"],
)

py_proto(
    name = "ground_service_detector_config_pb2",
    src = "ground_service_detector_config.proto",
)
