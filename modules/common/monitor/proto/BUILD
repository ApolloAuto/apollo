package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "monitor_proto",
    deps = [
        ":monitor_proto_lib",
    ],
)

proto_library(
    name = "monitor_proto_lib",
    srcs = [
        "monitor.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
