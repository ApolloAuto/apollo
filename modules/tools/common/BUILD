load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "logger",
    srcs = ["logger.py"],
)

py_library(
    name = "message_manager",
    srcs = ["message_manager.py"],
    deps = [
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/common/proto:drive_state_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
        "//modules/guardian/proto:guardian_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/map/relative_map/proto:navigation_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
        "//modules/perception/proto:traffic_light_detection_py_pb2",
        "//modules/planning/proto:planning_internal_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/prediction/proto:prediction_obstacle_py_pb2",
        "//modules/routing/proto:routing_py_pb2",
    ],
)

py_library(
    name = "proto_utils",
    srcs = ["proto_utils.py"],
)
