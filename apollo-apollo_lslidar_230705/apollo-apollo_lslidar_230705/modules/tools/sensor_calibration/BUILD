load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = ["runtime_files"],
    data_dest = "tools/sensor_calibration",
    py_dest = "tools/sensor_calibration",
    targets = [
        ":extract_data",
        ":extract_static_data",
        ":ins_stat_publisher",
        ":odom_publisher",
        ":sanity_check",
    ],
    deps = ["//modules/tools/sensor_calibration/config:install"]

)

filegroup(
    name = "runtime_files",
    srcs = glob([
        #"config/**",
        "template/**",
        "*.py",
        "*.sh",
    ]),
)

py_library(
    name = "configuration_yaml_generator",
    srcs = ["configuration_yaml_generator.py"],
    deps = [
        "//modules/dreamview/proto:preprocess_table_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "data_file_object",
    srcs = ["data_file_object.py"],
)

py_binary(
    name = "extract_data",
    srcs = ["extract_data.py"],
    data = [":runtime_files"],
    deps = [
        ":configuration_yaml_generator",
        ":extract_static_data",
        ":sanity_check",
        ":sensor_msg_extractor",
        "//cyber/proto:record_py_pb2",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/dreamview/proto:preprocess_table_py_pb2",
        "//modules/tools/common:proto_utils",
        "//modules/tools/sensor_calibration/proto:extractor_config_py_pb2",
    ],
)

py_binary(
    name = "extract_static_data",
    srcs = ["extract_static_data.py"],
    data = [":runtime_files"],
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
        "//modules/common_msgs/sensor_msgs:ins_py_pb2",
    ],
)

py_binary(
    name = "odom_publisher",
    srcs = ["odom_publisher.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/localization_msgs:gps_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

py_library(
    name = "sensor_msg_extractor",
    srcs = ["sensor_msg_extractor.py"],
    deps = [
        ":data_file_object",
        "//modules/common_msgs/sensor_msgs:conti_radar_py_pb2",
        "//modules/common_msgs/sensor_msgs:pointcloud_py_pb2",
        "//modules/common_msgs/sensor_msgs:sensor_image_py_pb2",
        "//modules/common_msgs/localization_msgs:gps_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

py_binary(
    name = "sanity_check",
    srcs = ["sanity_check.py"],
    deps = [
        "//modules/tools/common:file_utils",
    ],
)
