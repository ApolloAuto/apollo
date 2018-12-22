load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stream_proto",
    deps = [
        ":config_proto",
        ":stream_status_proto",
    ],
)

cc_proto_library(
    name = "config_proto",
    deps = [
        ":config_lib",
    ],
)

proto_library(
    name = "config_lib",
    srcs = ["config.proto"],
    deps = [
    ],
)


cc_proto_library(
    name = "stream_status_proto",
    deps = [
        ":stream_status_proto_lib",
    ],
)

proto_library(
    name = "stream_status_proto_lib",
    srcs = [
        "stream_status.proto",
    ],
    deps = [
       # "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
