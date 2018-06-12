package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "static_info_proto",
    deps = [":static_info_proto_lib"],
)

proto_library(
    name = "static_info_proto_lib",
    srcs = ["static_info.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/configs/proto:vehicle_config_proto_lib",
        "//modules/control/proto:control_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

proto_library(
    name = "recorder_info_proto_lib",
    srcs = ["recorder_info.proto"],
)

proto_library(
    name = "task_proto_lib",
    srcs = ["task.proto"],
    deps = [
        ":static_info_proto_lib",
    ],
)

proto_library(
    name = "warehouse_query_proto_lib",
    srcs = ["warehouse_query.proto"],
    deps = [
        ":task_proto_lib",
    ],
)
