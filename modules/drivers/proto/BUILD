load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "sensor_proto",
    deps = [
        ":sensor_proto_lib",
    ],
)

proto_library(
    name = "sensor_proto_lib",
    srcs = [
        "conti_radar.proto",
        "delphi_esr.proto",
        "mobileye.proto",
        "pointcloud.proto",
        "racobit_radar.proto",
        "radar.proto",
        "sensor_image.proto",
        "ultrasonic_radar.proto",
    ],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_proto_lib",
        "//modules/drivers/radar/racobit_radar/proto:racobit_radar_proto_lib",
        "//modules/drivers/radar/ultrasonic_radar/proto:ultrasonic_radar_proto_lib",
    ],
)

py_proto(
    name = "sensor_image_pb2",
    src = "sensor_image.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "racobit_radar_pb2",
    src = "racobit_radar.proto",
    deps = [
        "//modules/common/proto:header_pb2",
        "//modules/drivers/radar/racobit_radar/proto:racobit_radar_conf_pb2",
    ],
)

py_proto(
    name = "mobileye_pb2",
    src = "mobileye.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "delphi_esr_pb2",
    src = "delphi_esr.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "conti_radar_pb2",
    src = "conti_radar.proto",
    deps = [
        "//modules/common/proto:header_pb2",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_pb2",
    ],
)

py_proto(
    name = "pointcloud_pb2",
    src = "pointcloud.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "radar_pb2",
    src = "radar.proto",
    deps = [
        "//modules/common/proto:error_code_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "ultrasonic_radar_pb2",
    src = "ultrasonic_radar.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)
