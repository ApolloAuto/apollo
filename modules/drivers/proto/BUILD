load("//tools:cpplint.bzl", "cpplint")

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
