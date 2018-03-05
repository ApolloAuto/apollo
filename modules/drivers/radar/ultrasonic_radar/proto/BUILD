package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "ultrasonic_radar_proto",
    deps = [
        ":ultrasonic_radar_proto_lib",
    ],
)

proto_library(
    name = "ultrasonic_radar_proto_lib",
    srcs = [
        "ultrasonic_radar_conf.proto",
    ],
    deps = [
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
