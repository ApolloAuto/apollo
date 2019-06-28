load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacles_container",
    srcs = ["obstacles_container.cc"],
    hdrs = ["obstacles_container.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:environment_features",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_constants",
        "//modules/prediction/container",
        "//modules/prediction/container/obstacles:obstacle",
    ],
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/filters:digital_filter",
        "//modules/prediction/common:junction_analyzer",
        "//modules/prediction/container/obstacles:obstacle_clusters",
        "//modules/prediction/network/rnn_model",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_test(
    name = "obstacles_container_test",
    size = "small",
    srcs = [
        "obstacles_container_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "@gtest//:main",
    ],
)

cc_test(
    name = "obstacle_test",
    size = "small",
    srcs = [
        "obstacle_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "@gtest//:main",
    ],
)

cc_library(
    name = "obstacle_clusters",
    srcs = [
        "obstacle_clusters.cc",
    ],
    hdrs = [
        "obstacle_clusters.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:road_graph",
        "//modules/prediction/proto:feature_proto",
    ],
)

cc_test(
    name = "obstacle_clusters_test",
    size = "small",
    srcs = [
        "obstacle_clusters_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacle_clusters",
        "@gtest//:main",
    ],
)

cpplint()
