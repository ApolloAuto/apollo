load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_sequence_predictor",
    srcs = ["move_sequence_predictor.cc"],
    hdrs = ["move_sequence_predictor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/math:geometry",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
        "//modules/prediction/predictor/sequence:sequence_predictor",
        "//modules/prediction/proto:lane_graph_proto",
        "@eigen",
    ],
)

cc_test(
    name = "move_sequence_predictor_test",
    size = "small",
    srcs = [
        "move_sequence_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/util",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/predictor/move_sequence:move_sequence_predictor",
        "//modules/prediction/proto:prediction_proto",
        "@gtest//:main",
    ],
)

cpplint()
