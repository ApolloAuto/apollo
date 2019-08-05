package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "tick_conf_proto",
    deps = [
        ":tick_conf_proto_lib",
    ],
)

proto_library(
    name = "tick_conf_proto_lib",
    srcs = [
        "tick_conf.proto",
    ],
)

cc_proto_library(
    name = "tick_proto",
    deps = [
        ":tick_proto_lib",
    ],
)

proto_library(
    name = "tick_proto_lib",
    srcs = [
        "tick.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)
