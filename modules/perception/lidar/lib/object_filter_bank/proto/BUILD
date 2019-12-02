load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "filter_bank_config_proto",
    deps = [
        ":filter_bank_config_proto_lib",
    ],
)

proto_library(
    name = "filter_bank_config_proto_lib",
    srcs = ["filter_bank_config.proto"],
)

cpplint()
