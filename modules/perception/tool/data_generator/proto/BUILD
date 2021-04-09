package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "config_proto",
    deps = [
        ":config_proto_lib",
    ],
)

proto_library(
    name = "config_proto_lib",
    srcs = [
        "config.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)
