load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "suteng_proto",
    deps = [
        ":sensor_suteng_proto",
    ],
)

cc_proto_library(
    name = "sensor_suteng_proto",
    deps = [
        ":sensor_suteng_conf_proto_lib",
    ],
)

proto_library(
    name = "sensor_suteng_conf_proto_lib",
    srcs = [
        "sensor_suteng_conf.proto",
        "lidars_filter_config.proto",
    ],
    deps = [
        ":sensor_suteng_proto_lib",
    ],
)

proto_library(
    name = "sensor_suteng_proto_lib",
    srcs = [
        "sensor_suteng.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

cpplint()
