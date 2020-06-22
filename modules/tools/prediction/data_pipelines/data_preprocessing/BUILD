load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "combine_features_and_labels",
    srcs = ["combine_features_and_labels.py"],
    deps = [
        ":features_labels_utils",
    ],
)

py_library(
    name = "combine_features_and_labels_for_junction",
    srcs = ["combine_features_and_labels_for_junction.py"],
    deps = [
        ":features_labels_utils",
    ],
)

py_library(
    name = "features_labels_utils",
    srcs = ["features_labels_utils.py"],
    deps = [
        "//modules/prediction/proto:offline_features_py_pb2",
    ],
)

py_binary(
    name = "generate_cruise_labels",
    srcs = ["generate_cruise_labels.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:online_to_offline",
    ],
)

py_binary(
    name = "generate_future_trajectory",
    srcs = ["generate_future_trajectory.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:online_to_offline",
    ],
)

py_binary(
    name = "generate_junction_labels",
    srcs = ["generate_junction_labels.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:online_to_offline",
    ],
)

py_binary(
    name = "generate_labels",
    srcs = ["generate_labels.py"],
    deps = [
        "//modules/tools/prediction/data_pipelines/common:online_to_offline",
    ],
)

py_binary(
    name = "merge_label_dicts",
    srcs = ["merge_label_dicts.py"],
    deps = [
        ":features_labels_utils",
    ],
)
