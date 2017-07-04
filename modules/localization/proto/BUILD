load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "localization_config_proto",
    protos = ["localization_config.proto"],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "localization_config_proto_pylib",
    protos = ["localization_config.proto"],
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)

cc_proto_library(
    name = "pose_proto",
    protos = ["pose.proto"],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "pose_proto_pylib",
    protos = ["pose.proto"],
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)

cc_proto_library(
    name = "gps_proto",
    protos = ["gps.proto"],
    deps = [
        ":pose_proto",
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "gps_proto_pylib",
    protos = ["gps.proto"],
    deps = [
        ":pose_proto_pylib",
        "//modules/common/proto:common_proto_pylib",
    ],
)

cc_proto_library(
    name = "imu_proto",
    protos = ["imu.proto"],
    deps = [
        ":pose_proto",
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "imu_proto_pylib",
    protos = ["imu.proto"],
    deps = [
        ":pose_proto_pylib",
        "//modules/common/proto:common_proto_pylib",
    ],
)

cc_proto_library(
    name = "camera_proto",
    protos = ["camera.proto"],
    deps = [
        ":pose_proto",
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "camera_proto_pylib",
    protos = ["camera.proto"],
    deps = [
        ":pose_proto_pylib",
        "//modules/common/proto:common_proto_pylib",
    ],
)

cc_proto_library(
    name = "camera_parameter_proto",
    protos = ["camera_parameter.proto"],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "camera_parameter_proto_pylib",
    protos = ["camera_parameter.proto"],
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)

cc_proto_library(
    name = "localization_proto",
    protos = ["localization.proto"],
    deps = [
        ":pose_proto",
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "localization_proto_pylib",
    protos = ["localization.proto"],
    deps = [
        ":pose_proto_pylib",
        "//modules/common/proto:common_proto_pylib",
    ],
)

cpplint()
