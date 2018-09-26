load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "probabilistic_fusion_config_proto",
    deps = [
        ":probabilistic_fusion_config_proto_lib",
    ],
)

proto_library(
    name = "probabilistic_fusion_config_proto_lib",
    srcs = [
        "probabilistic_fusion_config.proto",
    ],
    deps = [
    ],
)

cpplint()
