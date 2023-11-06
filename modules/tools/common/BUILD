load("//tools/proto:proto.bzl", "apollo_py_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# filegroup(
#     name = "py_files",
#     srcs = glob([
#         "*.py",
#     ]),
# )

apollo_py_library(
    name = "logger",
    srcs = ["logger.py"],
)

apollo_py_library(
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

apollo_py_library(
    name = "proto_utils",
    srcs = ["proto_utils.py"],
)

apollo_py_library(
    name = "file_utils",
    srcs = ["file_utils.py"],
)

apollo_package()