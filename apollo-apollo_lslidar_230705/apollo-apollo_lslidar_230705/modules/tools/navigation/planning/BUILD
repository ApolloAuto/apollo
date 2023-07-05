load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "ad_vehicle",
    srcs = ["ad_vehicle.py"],
)

py_library(
    name = "heading_decider",
    srcs = ["heading_decider.py"],
)

py_library(
    name = "lanemarker_corrector",
    srcs = ["lanemarker_corrector.py"],
)

py_library(
    name = "local_path",
    srcs = ["local_path.py"],
)

py_library(
    name = "provider_chassis",
    srcs = ["provider_chassis.py"],
)

py_library(
    name = "provider_localization",
    srcs = ["provider_localization.py"],
)

py_library(
    name = "reference_path",
    srcs = ["reference_path.py"],
)

py_library(
    name = "speed_decider",
    srcs = ["speed_decider.py"],
)

py_library(
    name = "trajectory_generator",
    srcs = ["trajectory_generator.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_state_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)
