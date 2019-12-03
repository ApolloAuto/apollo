load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "ground_service_config_proto",
    deps = [
        ":ground_service_config_proto_lib",
    ],
)

proto_library(
    name = "ground_service_config_proto_lib",
    srcs = ["ground_service_config.proto"],
)

cpplint()
