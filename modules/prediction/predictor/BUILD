load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "predictor_manager",
    srcs = ["predictor_manager.cc"],
    hdrs = ["predictor_manager.h"],
    deps = [
        "//modules/common:macro",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor/empty:empty_predictor",
        "//modules/prediction/predictor/free_move:free_move_predictor",
        "//modules/prediction/predictor/lane_sequence:lane_sequence_predictor",
        "//modules/prediction/predictor/move_sequence:move_sequence_predictor",
        "//modules/prediction/predictor/regional:regional_predictor",
        "//modules/prediction/predictor/single_lane:single_lane_predictor",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/proto:prediction_proto",
        "@glog",
    ],
)

cc_test(
    name = "predictor_manager_test",
    size = "small",
    srcs = ["predictor_manager_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/common/util",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:prediction_conf_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "predictor",
    srcs = ["predictor.cc"],
    hdrs = ["predictor.h"],
    deps = [
        "//modules/common/math:geometry",
        "//modules/common/proto:pnc_point_proto",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/proto:prediction_proto",
        "@eigen",
    ],
)

cpplint()
