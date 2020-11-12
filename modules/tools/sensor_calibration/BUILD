load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob([
        "config/**",
    ]),
)

py_library(
    name = "configuration_yaml_generator",
    srcs = ["configuration_yaml_generator.py"],
)

py_library(
    name = "data_file_object",
    srcs = ["data_file_object.py"],
)

py_binary(
    name = "extract_data",
    srcs = ["extract_data.py"],
    data = [":config"],
    deps = [
        ":configuration_yaml_generator",
        ":extract_static_data",
        ":sensor_msg_extractor",
        "//cyber/proto:record_py_pb2",
        "//cyber/python/cyber_py3:record",
        "//modules/tools/sensor_calibration/proto:extractor_config_py_pb2",
    ],
)

py_binary(
    name = "extract_static_data",
    srcs = ["extract_static_data.py"],
    data = [":config"],
    deps = [
        ":configuration_yaml_generator",
        ":data_file_object",
        "//cyber/proto:record_py_pb2",
        "//cyber/python/cyber_py3:record",
        "//modules/tools/sensor_calibration/proto:extractor_config_py_pb2",
    ],
)

py_binary(
    name = "ins_stat_publisher",
    srcs = ["ins_stat_publisher.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/drivers/gnss/proto:ins_py_pb2",
    ],
)

py_binary(
    name = "odom_publisher",
    srcs = ["odom_publisher.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/localization/proto:gps_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
    ],
)

py_library(
    name = "sensor_msg_extractor",
    srcs = ["sensor_msg_extractor.py"],
    deps = [
        ":data_file_object",
        "//modules/drivers/proto:conti_radar_py_pb2",
        "//modules/drivers/proto:pointcloud_py_pb2",
        "//modules/drivers/proto:sensor_image_py_pb2",
        "//modules/localization/proto:gps_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
    ],
)
