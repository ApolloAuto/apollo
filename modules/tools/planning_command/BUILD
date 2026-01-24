load("//tools/proto:proto.bzl", "apollo_py_binary")
load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "planning_command",
    srcs = ["planning_command.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/planning_msgs:planning_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:free_space_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:action_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:external_geometry_py_pb2",
        "//modules/common_msgs/external_command_msgs:precise_parking_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:zone_cover_command_py_pb2",
        "//modules/common_msgs/routing_msgs:routing_py_pb2",
        "//modules/common_msgs/external_command_msgs:command_status_py_pb2",
        "//modules/tools/common:proto_utils"
    ],
)

apollo_package()
