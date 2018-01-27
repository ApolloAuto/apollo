package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "data_generator_proto",
    deps = [
        ":data_generator_proto_lib",
    ],
)

proto_library(
    name = "data_generator_proto_lib",
    srcs = [
        "data_generator.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)
