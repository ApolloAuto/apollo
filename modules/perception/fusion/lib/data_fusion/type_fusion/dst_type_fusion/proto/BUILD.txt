load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "dst_type_fusion_config_proto",
    deps = [
        ":dst_type_fusion_config_proto_lib",
    ],
)

proto_library(
    name = "dst_type_fusion_config_proto_lib",
    srcs = [
        "dst_type_fusion_config.proto",
    ],
    deps = [
    ],
)

cpplint()
