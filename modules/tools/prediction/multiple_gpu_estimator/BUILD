load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "convert_to_tfrecords",
    srcs = ["convert_to_tfrecords.py"],
)

apollo_py_binary(
    name = "counting",
    srcs = ["counting.py"],
)

apollo_py_library(
    name = "mlp_data",
    srcs = ["mlp_data.py"],
)

apollo_py_binary(
    name = "mlp_main",
    srcs = ["mlp_main.py"],
    deps = [
        ":mlp_data",
        ":mlp_model",
        ":mlp_utils",
    ],
)

apollo_py_library(
    name = "mlp_model",
    srcs = ["mlp_model.py"],
    deps = [
        ":model_base",
    ],
)

apollo_py_library(
    name = "mlp_utils",
    srcs = ["mlp_utils.py"],
)

apollo_py_library(
    name = "model_base",
    srcs = ["model_base.py"],
)

apollo_py_binary(
    name = "preprocessing",
    srcs = ["preprocessing.py"],
)

apollo_package()