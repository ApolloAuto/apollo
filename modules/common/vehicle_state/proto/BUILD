package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "vehicle_state_proto",
    deps = [
        ":vehicle_state_proto_lib",
    ],
)

proto_library(
    name = "vehicle_state_proto_lib",
    srcs = ["vehicle_state.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/localization/proto:pose_proto_lib",
    ],
)
