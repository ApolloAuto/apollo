load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_library(
    name = "generator",
    srcs = ["generator.py"],
    deps = [
        "//modules/common_msgs/planning_msgs:navigation_py_pb2",
    ],
)

apollo_py_binary(
    name = "viewer",
    srcs = ["viewer.py"],
    deps = [
        "//modules/common_msgs/planning_msgs:navigation_py_pb2",
    ],
)

apollo_package()