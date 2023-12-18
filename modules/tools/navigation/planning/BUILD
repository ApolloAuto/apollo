load("//tools/proto:proto.bzl", "apollo_py_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_library(
    name = "ad_vehicle",
    srcs = ["ad_vehicle.py"],
)

apollo_py_library(
    name = "heading_decider",
    srcs = ["heading_decider.py"],
)

apollo_py_library(
    name = "lanemarker_corrector",
    srcs = ["lanemarker_corrector.py"],
)

apollo_py_library(
    name = "local_path",
    srcs = ["local_path.py"],
)

apollo_py_library(
    name = "provider_chassis",
    srcs = ["provider_chassis.py"],
)

apollo_py_library(
    name = "provider_localization",
    srcs = ["provider_localization.py"],
)

apollo_py_library(
    name = "reference_path",
    srcs = ["reference_path.py"],
)

apollo_py_library(
    name = "speed_decider",
    srcs = ["speed_decider.py"],
)

apollo_py_library(
    name = "trajectory_generator",
    srcs = ["trajectory_generator.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/basic_msgs:drive_state_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

apollo_package()