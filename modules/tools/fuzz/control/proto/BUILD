package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "control_fuzz_proto",
    deps = [
        ":control_fuzz_proto_lib",
    ],
)

cc_proto_library(
    name = "control_module_fuzz_proto",
    deps = [
        ":control_module_fuzz_proto_lib",
    ],
)

proto_library(
    name = "control_fuzz_proto_lib",
    srcs = [
        "control_message.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/control/proto:control_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/planning/proto:planning_proto_lib",
    ],
)

proto_library(
    name = "control_module_fuzz_proto_lib",
    srcs = [
        "control_fuzz.proto",
    ],
    deps = [
        "//modules/common/monitor_log/proto:monitor_log_proto_lib", 
        "//modules/control/proto:control_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/planning/proto:planning_proto_lib",
    ],
)
