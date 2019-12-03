package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "static_transform_conf_proto",
    deps = [
        ":static_transform_conf_proto_lib",
    ],
)

proto_library(
    name = "static_transform_conf_proto_lib",
    srcs = ["static_transform_conf.proto"],
)

cc_proto_library(
    name = "transform_proto",
    deps = [
        ":transform_proto_lib",
    ],
)

proto_library(
    name = "transform_proto_lib",
    srcs = ["transform.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
