package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "racobit_radar_proto",
    deps = [
        ":racobit_radar_proto_lib",
    ],
)

proto_library(
    name = "racobit_radar_proto_lib",
    srcs = ["racobit_radar_conf.proto"],
    deps = [
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
