load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "velodyne_cc_proto",
    deps = [
        ":velodyne_proto",
    ],
)

proto_library(
    name = "velodyne_proto",
    srcs = ["velodyne.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "velodyne_py_pb2",
    deps = [
        ":velodyne_proto",
        "//modules/common/proto:header_py_pb2",
    ],
)

cc_proto_library(
    name = "velodyne_config_cc_proto",
    deps = [
        ":velodyne_config_proto",
    ],
)

proto_library(
    name = "velodyne_config_proto",
    srcs = ["velodyne_config.proto"],
    deps = [
        ":velodyne_proto",
    ],
)

##hesai
py_proto_library(
    name = "hesai_py_pb2",
    deps = [
        ":hesai_proto",
        "//modules/common/proto:header_py_pb2",
    ],
)

proto_library(
    name = "hesai_proto",
    srcs = ["hesai.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

cc_proto_library(
    name = "hesai_cc_proto",
    deps = [
        ":hesai_proto",
    ],
)

proto_library(
    name = "hesai_config_proto",
    srcs = ["hesai_config.proto"],
    deps = [
        ":hesai_proto",
    ],
)

cc_proto_library(
    name = "hesai_config_cc_proto",
    deps = [
        ":hesai_config_proto",
    ],
)

##robosense
cc_library(
    name = "robosense_proto",
    deps = [
        ":sensor_robosense_proto",
    ],
)

cc_proto_library(
    name = "sensor_robosense_proto",
    deps = [
        ":robosense_proto_lib",
    ],
)

proto_library(
    name = "robosense_proto_lib",
    srcs = [
        "robosense.proto",
        "robosense_config.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

proto_library(
    name = "lidar_parameter_proto_lib",
    srcs = [
        "lidar_parameter.proto",
    ],
)

cc_proto_library(
    name = "lidar_parameter_proto",
    deps = [
        ":lidar_parameter_proto_lib",
    ],
)

cc_library(
    name = "lidar_parameter",
    deps = [
        ":lidar_parameter_proto",
    ],
)

proto_library(
    name = "config_proto_lib",
    srcs = [
        "config.proto",
    ],
    deps = [
        ":hesai_config_proto",
        ":lidar_parameter_proto_lib",
        ":robosense_proto_lib",
        ":velodyne_config_proto",
        "//modules/common/proto:header_proto",
    ],
)

cc_proto_library(
    name = "config_proto",
    deps = [
        ":config_proto_lib",
    ],
)

cc_library(
    name = "config",
    deps = [
        ":config_proto",
    ],
)

cpplint()
