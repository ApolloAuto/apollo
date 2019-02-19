package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "ultanalyse_proto",
    deps = [
        ":ultanalyse_proto_lib",
    ],
)

proto_library(
    name = "ultanalyse_proto_lib",
    srcs = [
        "ultanalyse.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
    ],
)
