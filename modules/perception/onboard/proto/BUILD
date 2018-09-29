load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "radar_component_config_proto",
    deps = [
        ":radar_component_config_proto_lib",
    ],
)

proto_library(
    name = "radar_component_config_proto_lib",
    srcs = [
        "radar_component_config.proto"
    ],
)

cpplint()
