package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "guardian_fuzz_proto",
    deps = [
        ":guardian_fuzz_proto_lib",
    ],
)

proto_library(
    name = "guardian_fuzz_proto_lib",
    srcs = [
        "guardian_fuzz.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/monitor/proto:system_status_proto_lib",
        "//modules/control/proto:control_proto_lib",
    ],
)