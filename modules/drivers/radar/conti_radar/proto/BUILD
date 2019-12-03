package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "conti_radar_proto",
    deps = [
        ":conti_radar_proto_lib",
    ],
)

proto_library(
    name = "conti_radar_proto_lib",
    srcs = ["conti_radar_conf.proto"],
    deps = [
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
