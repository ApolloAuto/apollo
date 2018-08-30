package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "control_fuzz_proto",
    deps = [
        ":control_fuzz_proto_lib",
    ],
)

proto_library(
    name = "control_fuzz_proto_lib",
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