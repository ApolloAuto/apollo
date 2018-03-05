package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "sonic_radar_proto",
    deps = [
        ":sonic_radar_proto_lib",
    ],
)

proto_library(
    name = "sonic_radar_proto_lib",
    srcs = [
        "sonic_radar_conf.proto",
    ],
    deps = [
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
