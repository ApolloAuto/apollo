load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_proto",
    deps = [
        ":config_proto",
        ":gnss_best_pose_proto",
        ":gnss_proto_proto",
        ":gnss_raw_observation_proto",
        ":gnss_status_proto",
        ":heading_proto",
        ":imu_proto",
        ":ins_proto",
    ],
)

cc_proto_library(
    name = "ins_proto",
    deps = [
        ":ins_proto_lib",
    ],
)

proto_library(
    name = "ins_proto_lib",
    srcs = ["ins.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_raw_observation_proto",
    deps = [
        ":gnss_raw_observation_lib",
    ],
)

proto_library(
    name = "gnss_raw_observation_lib",
    srcs = ["gnss_raw_observation.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_best_pose_proto",
    deps = [
        ":gnss_best_pose_lib",
    ],
)

proto_library(
    name = "gnss_best_pose_lib",
    srcs = ["gnss_best_pose.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
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
)

cc_proto_library(
    name = "imu_proto",
    deps = [
        ":imu_lib",
    ],
)

proto_library(
    name = "imu_lib",
    srcs = ["imu.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_status_proto",
    deps = [
        ":gnss_status_proto_lib",
    ],
)

proto_library(
    name = "gnss_status_proto_lib",
    srcs = ["gnss_status.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_proto_proto",
    deps = [
        ":gnss_proto_lib",
    ],
)

proto_library(
    name = "gnss_proto_lib",
    srcs = ["gnss.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "heading_proto",
    deps = [
        ":heading_proto_lib",
    ],
)

proto_library(
    name = "heading_proto_lib",
    srcs = ["heading.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "gnss_best_pose_pb2",
    src = "gnss_best_pose.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "gnss_pb2",
    src = "gnss.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "imu_pb2",
    src = "imu.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "ins_pb2",
    src = "ins.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "heading_pb2",
    src = "heading.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "config_pb2",
    src = "config.proto",
)

py_proto(
    name = "gnss_status_pb2",
    src = "gnss_status.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "gnss_raw_observation_pb2",
    src = "gnss_raw_observation.proto",
)
