load("//tools:cpplint.bzl", "cpplint")

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

cpplint()
