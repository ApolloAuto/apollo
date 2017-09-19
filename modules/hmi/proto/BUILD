package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "config_proto",
    deps = [":config_proto_lib"],
)

proto_library(
    name = "config_proto_lib",
    srcs = ["config.proto"],
)

cc_proto_library(
    name = "runtime_status_proto",
    deps = [":runtime_status_proto_lib"],
)

proto_library(
    name = "runtime_status_proto_lib",
    srcs = ["runtime_status.proto"],
)

cc_proto_library(
    name = "hmi_message_proto",
    deps = [":hmi_message_proto_lib"],
)

proto_library(
    name = "hmi_message_proto_lib",
    srcs = ["hmi_message.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
