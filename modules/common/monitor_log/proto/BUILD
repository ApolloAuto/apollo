package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "monitor_log_proto",
    deps = [
        ":monitor_log_proto_lib",
    ],
)

proto_library(
    name = "monitor_log_proto_lib",
    srcs = ["monitor_log.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
