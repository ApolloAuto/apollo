load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "distribution_analyzer",
    srcs = ["distribution_analyzer.py"],
    deps = [
        ":statistical_analyzer",
    ],
)

py_library(
    name = "error_code_analyzer",
    srcs = ["error_code_analyzer.py"],
    deps = [
        ":distribution_analyzer",
        ":statistical_analyzer",
        "//modules/common/proto:error_code_py_pb2",
    ],
)

py_library(
    name = "error_msg_analyzer",
    srcs = ["error_msg_analyzer.py"],
    deps = [
        ":distribution_analyzer",
        ":statistical_analyzer",
        "//modules/common/proto:error_code_py_pb2",
    ],
)

py_binary(
    name = "frechet_distance",
    srcs = ["frechet_distance.py"],
)

py_library(
    name = "statistical_analyzer",
    srcs = ["statistical_analyzer.py"],
)
