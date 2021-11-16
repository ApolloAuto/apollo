load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "convert_to_tfrecords",
    srcs = ["convert_to_tfrecords.py"],
)

py_binary(
    name = "counting",
    srcs = ["counting.py"],
)

py_library(
    name = "mlp_data",
    srcs = ["mlp_data.py"],
)

py_binary(
    name = "mlp_main",
    srcs = ["mlp_main.py"],
    deps = [
        ":mlp_data",
        ":mlp_model",
        ":mlp_utils",
    ],
)

py_library(
    name = "mlp_model",
    srcs = ["mlp_model.py"],
    deps = [
        ":model_base",
    ],
)

py_library(
    name = "mlp_utils",
    srcs = ["mlp_utils.py"],
)

py_library(
    name = "model_base",
    srcs = ["model_base.py"],
)

py_binary(
    name = "preprocessing",
    srcs = ["preprocessing.py"],
)
