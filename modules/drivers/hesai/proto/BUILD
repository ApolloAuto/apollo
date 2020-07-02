load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hesai_proto",
    deps = [
        ":sensor_hesai_proto",
    ],
)

cc_proto_library(
    name = "sensor_hesai_proto",
    deps = [
        ":hesai_proto_lib",
    ],
)

proto_library(
    name = "hesai_proto_lib",
    srcs = [
        "config.proto",
        "hesai.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
