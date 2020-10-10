load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "bounding_rectangle",
    srcs = ["bounding_rectangle.py"],
    deps = [
        ":rotation2d",
        ":util",
        ":vector2d",
    ],
)

py_library(
    name = "configure",
    srcs = ["configure.py"],
)

py_library(
    name = "data_preprocess",
    srcs = ["data_preprocess.py"],
)

py_library(
    name = "feature_io",
    srcs = ["feature_io.py"],
    deps = [
        "//modules/prediction/proto:feature_py_pb2",
        "//modules/prediction/proto:offline_features_py_pb2",
    ],
)

py_library(
    name = "log",
    srcs = ["log.py"],
)

py_library(
    name = "online_to_offline",
    srcs = ["online_to_offline.py"],
    deps = [
        ":bounding_rectangle",
        ":configure",
        "//modules/prediction/proto:offline_features_py_pb2",
    ],
)

py_library(
    name = "rotation2d",
    srcs = ["rotation2d.py"],
    deps = [
        ":vector2d",
    ],
)

py_library(
    name = "trajectory",
    srcs = ["trajectory.py"],
    deps = [
        ":bounding_rectangle",
        ":configure",
    ],
)

py_library(
    name = "util",
    srcs = ["util.py"],
)

py_library(
    name = "vector2d",
    srcs = ["vector2d.py"],
)
