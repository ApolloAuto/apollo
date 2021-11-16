load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "cruise_h5_preprocessing",
    srcs = ["cruise_h5_preprocessing.py"],
)

py_library(
    name = "cruise_models",
    srcs = ["cruise_models.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:configure",
        "//modules/tools/prediction/data_pipelines/proto:cruise_model_py_pb2",
    ],
)

py_binary(
    name = "cruiseMLP_train",
    srcs = ["cruiseMLP_train.py"],
    deps = [
        ":cruise_models",
        "//modules/tools/prediction/data_pipelines/common:configure",
        "//modules/tools/prediction/data_pipelines/proto:cruise_model_py_pb2",
    ],
)

py_binary(
    name = "junctionMLP_train",
    srcs = ["junctionMLP_train.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/proto:fnn_model_py_pb2",
    ],
)

py_binary(
    name = "merge_h5",
    srcs = ["merge_h5.py"],
)

py_binary(
    name = "mlp_train",
    srcs = ["mlp_train.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:configure",
        "//modules/tools/prediction/data_pipelines/common:data_preprocess",
        "//modules/tools/prediction/data_pipelines/common:log",
        "//modules/tools/prediction/data_pipelines/proto:fnn_model_py_pb2",
    ],
)
