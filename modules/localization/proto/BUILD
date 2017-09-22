load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "localization_config_proto",
    deps = [
        ":localization_config_proto_lib",
    ],
)

proto_library(
    name = "localization_config_proto_lib",
    srcs = ["localization_config.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)

cc_proto_library(
    name = "pose_proto",
    deps = [
        ":pose_proto_lib",
    ],
)

proto_library(
    name = "pose_proto_lib",
    srcs = ["pose.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)

cc_proto_library(
    name = "gps_proto",
    deps = [
        ":gps_proto_lib",
    ],
)

proto_library(
    name = "gps_proto_lib",
    srcs = ["gps.proto"],
    deps = [
        ":pose_proto_lib",
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "imu_proto",
    deps = [
        ":imu_proto_lib",
    ],
)

proto_library(
    name = "imu_proto_lib",
    srcs = ["imu.proto"],
    deps = [
        ":pose_proto_lib",
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "camera_proto",
    deps = [
        ":camera_proto_lib",
    ],
)

proto_library(
    name = "camera_proto_lib",
    srcs = ["camera.proto"],
    deps = [
        ":pose_proto_lib",
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "camera_parameter_proto",
    deps = [
        ":camera_parameter_proto_lib",
    ],
)

proto_library(
    name = "camera_parameter_proto_lib",
    srcs = ["camera_parameter.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)

cc_proto_library(
    name = "localization_proto",
    deps = [
        ":localization_proto_lib",
    ],
)

proto_library(
    name = "localization_proto_lib",
    srcs = ["localization.proto"],
    deps = [
        ":pose_proto_lib",
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
