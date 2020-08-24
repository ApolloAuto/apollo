load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "surestar_proto",
    deps = [
        ":sensor_surestar_proto",
    ],
)

cc_proto_library(
    name = "sensor_surestar_proto",
    deps = [
        ":sensor_surestar_conf_proto_lib",
    ],
)

proto_library(
    name = "sensor_surestar_conf_proto_lib",
    srcs = [
        "sensor_surestar_conf.proto",
        "lidars_filter_config.proto",
    ],
    deps = [
        ":sensor_surestar_proto_lib",
    ],
)

proto_library(
    name = "sensor_surestar_proto_lib",
    srcs = [
        "sensor_surestar.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
