load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob(["*.yaml"]),
)

py_library(
    name = "parse_camera",
    srcs = ["parse_camera.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/drivers/proto:sensor_image_py_pb2",
    ],
)

py_library(
    name = "parse_lidar",
    srcs = ["parse_lidar.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/drivers/proto:pointcloud_py_pb2",
    ],
)

py_library(
    name = "parse_radar",
    srcs = ["parse_radar.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/drivers/proto:conti_radar_py_pb2",
    ],
)

py_binary(
    name = "record_parse_save",
    srcs = ["record_parse_save.py"],
    data = [":config"],
    deps = [
        ":parse_camera",
        ":parse_lidar",
        ":parse_radar",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
    ],
)
