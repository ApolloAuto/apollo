load("//tools:cpplint.bzl", "cpplint")

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
