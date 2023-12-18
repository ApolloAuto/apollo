load("//tools/proto:proto.bzl", "apollo_py_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_library(
    name = "bounding_rectangle",
    srcs = ["bounding_rectangle.py"],
    deps = [
        ":rotation2d",
        ":util",
        ":vector2d",
    ],
)

apollo_py_library(
    name = "configure",
    srcs = ["configure.py"],
)

apollo_py_library(
    name = "data_preprocess",
    srcs = ["data_preprocess.py"],
)

apollo_py_library(
    name = "feature_io",
    srcs = ["feature_io.py"],
    deps = [
        "//modules/common_msgs/prediction_msgs:feature_py_pb2",
        "//modules/prediction/proto:offline_features_py_pb2",
    ],
)

apollo_py_library(
    name = "log",
    srcs = ["log.py"],
)

apollo_py_library(
    name = "online_to_offline",
    srcs = ["online_to_offline.py"],
    deps = [
        ":bounding_rectangle",
        ":configure",
        "//modules/prediction/proto:offline_features_py_pb2",
    ],
)

apollo_py_library(
    name = "rotation2d",
    srcs = ["rotation2d.py"],
    deps = [
        ":vector2d",
    ],
)

apollo_py_library(
    name = "trajectory",
    srcs = ["trajectory.py"],
    deps = [
        ":bounding_rectangle",
        ":configure",
    ],
)

apollo_py_library(
    name = "util",
    srcs = ["util.py"],
)

apollo_py_library(
    name = "vector2d",
    srcs = ["vector2d.py"],
)

apollo_package()