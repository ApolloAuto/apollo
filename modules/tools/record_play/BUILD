load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "readme",
    srcs = glob([
        "README.md",
    ]),
)

py_binary(
    name = "rtk_player",
    srcs = ["rtk_player.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/config_msgs:vehicle_config_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_state_py_pb2",
        "//modules/common_msgs/basic_msgs:pnc_point_py_pb2",
        "//modules/common_msgs/control_msgs:pad_msg_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
        "//modules/tools/common:logger",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "rtk_recorder",
    srcs = ["rtk_recorder.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/tools/common:logger",
        "//modules/tools/common:proto_utils",
    ],
)

install(
    name = "install",
    data = [":readme"],
    data_dest = "tools/record_play",
    py_dest = "tools/record_play",
    targets = [
        ":rtk_player",
        ":rtk_recorder"
    ],
)