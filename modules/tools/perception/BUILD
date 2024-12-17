load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "garage_config",
    srcs = glob(["*.json"]) + ["garage_perception.bash"],
)

apollo_py_binary(
    name = "empty_prediction",
    srcs = ["empty_prediction.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_py_pb2",
    ],
)

apollo_py_binary(
    name = "extend_prediction",
    srcs = ["extend_prediction.py"],
    deps = [
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_binary(
    name = "print_perception",
    srcs = ["print_perception.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
    ],
)

apollo_py_binary(
    name = "replay_perception",
    srcs = ["replay_perception.py"],
    data = [":garage_config"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/basic_msgs:geometry_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
    ],
)

apollo_py_binary(
    name = "manual_perception",
    srcs = ["manual_perception.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
    ],
)

apollo_py_binary(
    name = "perception_road_edge_distance",
    srcs = ["perception_road_edge_distance.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

# install(
#     name = "install",
#     data_dest = "tools/perception",
#     py_dest = "tools/perception",
#     targets = [
#         ":empty_prediction",
#         ":extend_prediction",
#         ":print_perception",
#         ":replay_perception"
#     ]
# )

apollo_package()
