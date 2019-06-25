load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "ncut_param_proto_lib",
    srcs = [
        "ncut_config.proto",
        "ncut_param.proto",
    ],
)

cc_proto_library(
    name = "ncut_param_proto",
    deps = [
        ":ncut_param_proto_lib",
    ],
)

cpplint()
