load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_library(
    name = "distribution_analyzer",
    srcs = ["distribution_analyzer.py"],
    deps = [
        ":statistical_analyzer",
    ],
)

apollo_py_library(
    name = "error_code_analyzer",
    srcs = ["error_code_analyzer.py"],
    deps = [
        ":distribution_analyzer",
        ":statistical_analyzer",
        "//modules/common_msgs/basic_msgs:error_code_py_pb2",
    ],
)

apollo_py_library(
    name = "error_msg_analyzer",
    srcs = ["error_msg_analyzer.py"],
    deps = [
        ":distribution_analyzer",
        ":statistical_analyzer",
        "//modules/common_msgs/basic_msgs:error_code_py_pb2",
    ],
)

apollo_py_binary(
    name = "frechet_distance",
    srcs = ["frechet_distance.py"],
)

apollo_py_library(
    name = "statistical_analyzer",
    srcs = ["statistical_analyzer.py"],
)

# install(
#     name = "install",
#     py_dest = "tools/record_analyzer/common",
#     targets = [
#         ":frechet_distance",
#     ]
# )

apollo_package()