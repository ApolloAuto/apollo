package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "task_proto",
    deps = [":task_proto_lib"],
)

proto_library(
    name = "task_proto_lib",
    srcs = ["task.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/configs/proto:vehicle_config_proto_lib",
    ]
)
