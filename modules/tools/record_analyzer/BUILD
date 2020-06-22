load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "lidar_endtoend_analyzer",
    srcs = ["lidar_endtoend_analyzer.py"],
    deps = [
        "//modules/tools/record_analyzer/common:statistical_analyzer",
    ],
)

py_binary(
    name = "main",
    srcs = ["main.py"],
    deps = [
        ":lidar_endtoend_analyzer",
        ":module_control_analyzer",
        ":module_planning_analyzer",
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
        "//modules/drivers/proto:pointcloud_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/prediction/proto:prediction_obstacle_py_pb2",
    ],
)

py_library(
    name = "module_control_analyzer",
    srcs = ["module_control_analyzer.py"],
    deps = [
        "//modules/tools/record_analyzer/common:error_code_analyzer",
        "//modules/tools/record_analyzer/common:error_msg_analyzer",
        "//modules/tools/record_analyzer/common:statistical_analyzer",
    ],
)

py_library(
    name = "module_planning_analyzer",
    srcs = ["module_planning_analyzer.py"],
    deps = [
        "//modules/planning/proto:planning_py_pb2",
        "//modules/tools/record_analyzer/common:distribution_analyzer",
        "//modules/tools/record_analyzer/common:error_code_analyzer",
        "//modules/tools/record_analyzer/common:error_msg_analyzer",
        "//modules/tools/record_analyzer/common:frechet_distance",
        "//modules/tools/record_analyzer/common:statistical_analyzer",
        "//modules/tools/record_analyzer/metrics:curvature",
        "//modules/tools/record_analyzer/metrics:frame_count",
        "//modules/tools/record_analyzer/metrics:lat_acceleration",
        "//modules/tools/record_analyzer/metrics:latency",
        "//modules/tools/record_analyzer/metrics:lon_acceleration",
        "//modules/tools/record_analyzer/metrics:reference_line",
    ],
)
