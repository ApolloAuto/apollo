package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "ins_proto",
    deps = [
        ":ins_proto_lib",
    ],
)

proto_library(
    name = "ins_proto_lib",
    srcs = [
        "ins.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
