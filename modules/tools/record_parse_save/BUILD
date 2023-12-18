load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob(["*.yaml"]) + ["README.md"],
)

apollo_py_library(
    name = "parse_camera",
    srcs = ["parse_camera.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/sensor_msgs:sensor_image_py_pb2",
    ],
)

apollo_py_library(
    name = "parse_lidar",
    srcs = ["parse_lidar.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/sensor_msgs:pointcloud_py_pb2",
    ],
)

apollo_py_library(
    name = "parse_radar",
    srcs = ["parse_radar.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/sensor_msgs:conti_radar_py_pb2",
    ],
)

apollo_py_binary(
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

# install(
#     name = "install",
#     data = [":config"],
#     data_dest = "tools/record_parse_save",
#     py_dest = "tools/record_parse_save",
#     targets = [
#         ":record_parse_save",
#     ],
# )

apollo_package()