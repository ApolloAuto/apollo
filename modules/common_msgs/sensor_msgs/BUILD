load("//tools/proto:proto.bzl", "proto_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "delphi_esr_proto",
    srcs = ["delphi_esr.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "oculii_radar_proto",
    srcs = ["oculii_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        "//modules/common_msgs/basic_msgs:error_code_proto",
    ],
)

proto_library(
    name = "sensor_image_proto",
    srcs = ["sensor_image.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "mobileye_proto",
    srcs = ["mobileye.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "ultrasonic_radar_proto",
    srcs = ["ultrasonic_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "racobit_radar_proto",
    srcs = ["racobit_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "conti_radar_proto",
    srcs = ["conti_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "nano_radar_proto",
    srcs = ["nano_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "smartereye_proto",
    srcs = ["smartereye.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "pointcloud_proto",
    srcs = ["pointcloud.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "radarpointcloud_proto",
    srcs = ["radarpointcloud.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "radar_proto",
    srcs = ["radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_proto",
        "//modules/common_msgs/basic_msgs:geometry_proto",
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "gnss_best_pose_proto",
    srcs = ["gnss_best_pose.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "imu_proto",
    srcs = ["imu.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:geometry_proto",
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "heading_proto",
    srcs = ["heading.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "gnss_raw_observation_proto",
    srcs = ["gnss_raw_observation.proto"],
)

proto_library(
    name = "gnss_proto",
    srcs = ["gnss.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:geometry_proto",
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "ins_proto",
    srcs = ["ins.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:geometry_proto",
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

apollo_package()
