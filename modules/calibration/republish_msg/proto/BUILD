package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "relative_odometry_proto",
    deps = [
        "relative_odometry_proto_lib",
    ],
)

proto_library(
    name = "relative_odometry_proto_lib",
    srcs = [
        "relative_odometry.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
