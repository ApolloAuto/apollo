package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "trajectory_util_config_proto",
    deps = [
        ":trajectory_util_config_proto_lib",
    ],
)

proto_library(
    name = "trajectory_util_config_proto_lib",
    srcs = [
        "trajectory_util_config.proto",
    ],
    deps = [        
    ],
)

cc_proto_library(
    name = "trajectory_collector_proto",
    deps = [
        ":trajectory_collector_proto_lib",
    ],
)

proto_library(
    name = "trajectory_collector_proto_lib",
    srcs = [
        "trajectory_collector.proto",
    ],
     deps = [
    ],
)

cc_proto_library(
    name = "hmi_config_proto",
    deps = [
        ":hmi_config_proto_lib",
    ],
)

proto_library(
    name = "hmi_config_proto_lib",
    srcs = ["hmi_config.proto"],
)

cc_proto_library(
    name = "hmi_status_proto",
    deps = [
        ":hmi_status_proto_lib",
    ],
)

proto_library(
    name = "hmi_status_proto_lib",
    srcs = ["hmi_status.proto"],
    deps = [
        "//modules/monitor/proto:system_status_proto_lib",
    ],
)
