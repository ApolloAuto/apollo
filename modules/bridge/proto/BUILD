load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "udp_bridge_remote_info_proto",
    deps = [
        ":udp_bridge_remote_info_proto_lib",
    ],
)

proto_library(
    name = "udp_bridge_remote_info_proto_lib",
    srcs = ["udp_bridge_remote_info.proto"],
)

py_proto(
    name = "udp_bridge_remote_info_pb2",
    src = "udp_bridge_remote_info.proto",
)
