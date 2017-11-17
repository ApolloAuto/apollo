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
        "//modules/control/proto:control_proto_lib",
    ],
)

cc_proto_library(
    name = "recorder_conf_proto",
    deps = [":recorder_conf_proto_lib"],
)

proto_library(
    name = "recorder_conf_proto_lib",
    srcs = ["recorder_conf.proto"],
)

proto_library(
    name = "recorder_info_proto_lib",
    srcs = ["recorder_info.proto"],
)
