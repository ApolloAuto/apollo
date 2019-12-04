load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cnnseg_config_proto",
    deps = [
        ":cnnseg_config_proto_lib",
    ],
)

proto_library(
    name = "cnnseg_config_proto_lib",
    srcs = ["cnnseg_config.proto"],
)

cc_proto_library(
    name = "cnnseg_param_proto",
    deps = [
        ":cnnseg_param_proto_lib",
    ],
)

proto_library(
    name = "cnnseg_param_proto_lib",
    srcs = ["cnnseg_param.proto"],
)

cc_proto_library(
    name = "spp_engine_config_proto",
    deps = [
        ":spp_engine_config_proto_lib",
    ],
)

proto_library(
    name = "spp_engine_config_proto_lib",
    srcs = ["spp_engine_config.proto"],
)

cpplint()
