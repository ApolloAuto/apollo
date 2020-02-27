load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "scene_manager_config_proto",
    deps = [
        ":scene_manager_config_proto_lib",
    ],
)

proto_library(
    name = "scene_manager_config_proto_lib",
    srcs = ["scene_manager_config.proto"],
)

py_proto(
    name = "scene_manager_config_pb2",
    src = "scene_manager_config.proto",
)
