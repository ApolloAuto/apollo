package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "decision_proto",
    deps = [
        ":decision_proto_lib",
    ],
)

proto_library(
    name = "decision_proto_lib",
    srcs = [
        "decision.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
