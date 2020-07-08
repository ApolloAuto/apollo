load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "rtk_player",
    srcs = ["rtk_player.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/common/configs/proto:vehicle_config_py_pb2",
        "//modules/common/proto:drive_state_py_pb2",
        "//modules/common/proto:pnc_point_py_pb2",
        "//modules/control/proto:pad_msg_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "rtk_recorder",
    srcs = ["rtk_recorder.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)
