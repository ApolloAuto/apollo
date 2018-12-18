package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "simple_proto",
    deps = [
        ":simple_proto_lib",
    ],
)

proto_library(
    name = "simple_proto_lib",
    srcs = [
        "simple.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

filegroup(
    name = "data",
    srcs = glob([
        "hm_tracker_test/**/*",
    ]),
)
