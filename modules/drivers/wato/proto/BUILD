load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wato_proto",
    deps = [
        ":sensor_wato_proto",
    ],
)

cc_proto_library(
    name = "sensor_wato_proto",
    deps = [
        ":wato_proto_lib",
    ],
)

proto_library(
    name = "wato_proto_lib",
    srcs = ["config.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
