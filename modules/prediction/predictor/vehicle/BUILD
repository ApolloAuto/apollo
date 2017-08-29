load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_sequence_predictor",
    srcs = ["lane_sequence_predictor.cc"],
    hdrs = ["lane_sequence_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/math:math_utils",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
        "//modules/prediction/predictor",
        "//modules/prediction/proto:lane_graph_proto",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "free_move_predictor",
    srcs = ["free_move_predictor.cc"],
    hdrs = ["free_move_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/math:math_utils",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor",
        "//modules/prediction/proto:feature_proto",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "free_move_predictor_test",
    size = "small",
    srcs = [
        "free_move_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:road_graph",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/vehicle:free_move_predictor",
        "//modules/prediction/proto:prediction_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "lane_sequence_predictor_test",
    size = "small",
    srcs = [
        "lane_sequence_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/predictor/vehicle:lane_sequence_predictor",
        "//modules/prediction/proto:prediction_proto",
        "@gtest//:main",
    ],
)

cpplint()
