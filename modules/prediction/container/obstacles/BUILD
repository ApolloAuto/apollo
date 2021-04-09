load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacles_container",
    srcs = ["obstacles_container.cc"],
    hdrs = ["obstacles_container.h"],
    deps = [
        "//modules/common/math:geometry",
        "//modules/common/util:lru_cache",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/container/obstacles:obstacle_clusters",
        "//modules/prediction/container/pose:pose_container",
    ],
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/common/filters:digital_filter",
        "//modules/common/math:geometry",
        "//modules/common/math:kalman_filter",
        "//modules/common/proto:error_code_proto",
        "//modules/common/util:map_util",
        "//modules/map/hdmap",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:road_graph",
        "//modules/prediction/container/obstacles:obstacle_clusters",
        "//modules/prediction/network/rnn_model",
        "//modules/prediction/proto:feature_proto",
        "@eigen",
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
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
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
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
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
    deps = [
        "//modules/common:macro",
        "//modules/map/hdmap:hdmap_util",
        "//modules/prediction/common:road_graph",
        "//modules/prediction/proto:lane_graph_proto",
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
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/container/obstacles:obstacle_clusters",
        "@gtest//:main",
    ],
)

cpplint()
