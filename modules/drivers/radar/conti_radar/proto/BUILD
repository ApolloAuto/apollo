load("//tools:py_proto.bzl", "py_proto")

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

py_proto(
    name = "conti_radar_conf_pb2",
    src = "conti_radar_conf.proto",
    deps = [
        "//modules/drivers/canbus/proto:can_card_parameter_pb2",
    ],
)
