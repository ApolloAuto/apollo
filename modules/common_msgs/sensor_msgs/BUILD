load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "delphi_esr_cc_proto",
    deps = [
        ":delphi_esr_proto",
    ],
)

proto_library(
    name = "delphi_esr_proto",
    srcs = ["delphi_esr.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "delphi_esr_py_pb2",
    deps = [
        ":delphi_esr_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "sensor_image_cc_proto",
    deps = [
        ":sensor_image_proto",
    ],
)

proto_library(
    name = "sensor_image_proto",
    srcs = ["sensor_image.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "sensor_image_py_pb2",
    deps = [
        ":sensor_image_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "mobileye_cc_proto",
    deps = [
        ":mobileye_proto",
    ],
)

proto_library(
    name = "mobileye_proto",
    srcs = ["mobileye.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "mobileye_py_pb2",
    deps = [
        ":mobileye_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "ultrasonic_radar_cc_proto",
    deps = [
        ":ultrasonic_radar_proto",
    ],
)

proto_library(
    name = "ultrasonic_radar_proto",
    srcs = ["ultrasonic_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "ultrasonic_radar_py_pb2",
    deps = [
        ":ultrasonic_radar_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "racobit_radar_cc_proto",
    deps = [
        ":racobit_radar_proto",
    ],
)

proto_library(
    name = "racobit_radar_proto",
    srcs = ["racobit_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "racobit_radar_py_pb2",
    deps = [
        ":racobit_radar_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "conti_radar_cc_proto",
    deps = [
        ":conti_radar_proto",
    ],
)

proto_library(
    name = "conti_radar_proto",
    srcs = ["conti_radar.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "conti_radar_py_pb2",
    deps = [
        ":conti_radar_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "smartereye_cc_proto",
    deps = [
        ":smartereye_proto",
    ],
)

proto_library(
    name = "smartereye_proto",
    srcs = ["smartereye.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "smartereye_py_pb2",
    deps = [
        ":smartereye_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "pointcloud_cc_proto",
    deps = [
        ":pointcloud_proto",
    ],
)

proto_library(
    name = "pointcloud_proto",
    srcs = ["pointcloud.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "pointcloud_py_pb2",
    deps = [
        ":pointcloud_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "radar_cc_proto",
    deps = [
        ":radar_proto",
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

py_proto_library(
    name = "radar_py_pb2",
    deps = [
        ":radar_proto",
        "//modules/common_msgs/basic_msgs:error_code_py_pb2",
        "//modules/common_msgs/basic_msgs:geometry_py_pb2",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "gnss_best_pose_cc_proto",
    deps = [
        ":gnss_best_pose_proto",
    ],
)

proto_library(
    name = "gnss_best_pose_proto",
    srcs = ["gnss_best_pose.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "gnss_best_pose_py_pb2",
    deps = [
        ":gnss_best_pose_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "imu_cc_proto",
    deps = [
        ":imu_proto",
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

py_proto_library(
    name = "imu_py_pb2",
    deps = [
        ":imu_proto",
        "//modules/common_msgs/basic_msgs:geometry_py_pb2",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "heading_cc_proto",
    deps = [
        ":heading_proto",
    ],
)

proto_library(
    name = "heading_proto",
    srcs = ["heading.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "heading_py_pb2",
    deps = [
        ":heading_proto",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "gnss_raw_observation_cc_proto",
    deps = [
        ":gnss_raw_observation_proto",
    ],
)

proto_library(
    name = "gnss_raw_observation_proto",
    srcs = ["gnss_raw_observation.proto"],
)

py_proto_library(
    name = "gnss_raw_observation_py_pb2",
    deps = [
        ":gnss_raw_observation_proto",
    ],
)

cc_proto_library(
    name = "gnss_cc_proto",
    deps = [
        ":gnss_proto",
    ],
)

proto_library(
    name = "gnss_proto",
    srcs = ["gnss.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:geometry_proto",
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "gnss_py_pb2",
    deps = [
        ":gnss_proto",
        "//modules/common_msgs/basic_msgs:geometry_py_pb2",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)

cc_proto_library(
    name = "ins_cc_proto",
    deps = [
        ":ins_proto",
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

py_proto_library(
    name = "ins_py_pb2",
    deps = [
        ":ins_proto",
        "//modules/common_msgs/basic_msgs:geometry_py_pb2",
        "//modules/common_msgs/basic_msgs:header_py_pb2",
    ],
)
