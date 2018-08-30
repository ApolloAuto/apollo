package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "routing_fuzz_proto",
    deps = [
        ":routing_fuzz_proto_lib",
    ],
)

proto_library(
    name = "routing_fuzz_proto_lib",
    srcs = [
        "routing_fuzz.proto",
    ],
    deps = [
        "//modules/routing/proto:routing_proto_lib",
    ],
)