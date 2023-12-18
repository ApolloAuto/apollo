load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "cruise_h5_preprocessing",
    srcs = ["cruise_h5_preprocessing.py"],
)

apollo_py_library(
    name = "cruise_models",
    srcs = ["cruise_models.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:configure",
        "//modules/tools/prediction/data_pipelines/proto:cruise_model_py_pb2",
    ],
)

apollo_py_binary(
    name = "cruiseMLP_train",
    srcs = ["cruiseMLP_train.py"],
    deps = [
        ":cruise_models",
        "//modules/tools/prediction/data_pipelines/common:configure",
        "//modules/tools/prediction/data_pipelines/proto:cruise_model_py_pb2",
    ],
)

apollo_py_binary(
    name = "junctionMLP_train",
    srcs = ["junctionMLP_train.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/proto:fnn_model_py_pb2",
    ],
)

apollo_py_binary(
    name = "merge_h5",
    srcs = ["merge_h5.py"],
)

apollo_py_binary(
    name = "mlp_train",
    srcs = ["mlp_train.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:configure",
        "//modules/tools/prediction/data_pipelines/common:data_preprocess",
        "//modules/tools/prediction/data_pipelines/common:log",
        "//modules/tools/prediction/data_pipelines/proto:fnn_model_py_pb2",
    ],
)

apollo_package()