load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "py_files",
    srcs = glob([
        "*.py",
    ]),
)

py_library(
    name = "logger",
    srcs = ["logger.py"],
)

py_library(
    name = "message_manager",
    srcs = ["message_manager.py"],
    deps = [
        ":proto_utils",
        "//modules/common_msgs/audio_msgs:audio_event_py_pb2",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_event_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
        "//modules/common_msgs/guardian_msgs:guardian_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/planning_msgs:navigation_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_internal_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_py_pb2",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
    ],
)

py_library(
    name = "proto_utils",
    srcs = ["proto_utils.py"],
)

py_library(
    name = "file_utils",
    srcs = ["file_utils.py"],
)
