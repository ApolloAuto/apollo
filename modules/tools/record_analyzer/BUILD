load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "readme",
    srcs = [
        "README.md",
    ],
)

apollo_py_library(
    name = "lidar_endtoend_analyzer",
    srcs = ["lidar_endtoend_analyzer.py"],
    deps = [
        "//modules/tools/record_analyzer/common:statistical_analyzer",
    ],
)

apollo_py_binary(
    name = "record_analyzer",
    srcs = ["record_analyzer.py"],
    deps = [
        ":lidar_endtoend_analyzer",
        ":module_control_analyzer",
        ":module_planning_analyzer",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
        "//modules/common_msgs/sensor_msgs:pointcloud_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_py_pb2",
    ],
)

apollo_py_library(
    name = "module_control_analyzer",
    srcs = ["module_control_analyzer.py"],
    deps = [
        "//modules/tools/record_analyzer/common:error_code_analyzer",
        "//modules/tools/record_analyzer/common:error_msg_analyzer",
        "//modules/tools/record_analyzer/common:statistical_analyzer",
    ],
)

apollo_py_library(
    name = "module_planning_analyzer",
    srcs = ["module_planning_analyzer.py"],
    deps = [
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
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

# install(
#     name = "install",
#     data = [":readme"],
#     data_dest = "tools/record_analyzer",
#     py_dest = "tools/record_analyzer",
#     targets = [
#         ":main",
#     ],
#     deps = [
#         "//modules/tools/record_analyzer/tools:install",
#         "//modules/tools/record_analyzer/common:install",
#     ]
# )

apollo_package()