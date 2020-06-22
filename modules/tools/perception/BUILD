load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "garage_config",
    srcs = glob(["*.json"]),
)

py_binary(
    name = "empty_prediction",
    srcs = ["empty_prediction.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/prediction/proto:prediction_obstacle_py_pb2",
    ],
)

py_binary(
    name = "extend_prediction",
    srcs = ["extend_prediction.py"],
    deps = [
        "//modules/prediction/proto:prediction_obstacle_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "print_perception",
    srcs = ["print_perception.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/perception/proto:perception_obstacle_py_pb2",
    ],
)

py_binary(
    name = "replay_perception",
    srcs = ["replay_perception.py"],
    data = [":garage_config"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common/proto:geometry_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
    ],
)
