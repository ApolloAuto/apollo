load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "predictor_factory",
    hdrs = ["predictor_factory.h"],
    srcs = ["predictor_factory.cc"],
    deps = [
        "//modules/prediction/predictor:predictor",
        "//modules/prediction/predictor/vehicle:lane_sequence_predictor",
        "//modules/prediction/predictor/vehicle:free_move_predictor",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/common/util:factory",
        "//modules/common:macro",
        "@glog//:glog",
    ]
)

cc_library(
    name = "predictor_manager",
    srcs = ["predictor_manager.cc"],
    hdrs = ["predictor_manager.h"],
    deps = [
        "//modules/prediction/predictor:predictor",
        "//modules/prediction/predictor:predictor_factory",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/proto:prediction_proto",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/common:macro",
        "@glog//:glog"
    ]
)

cc_library(
    name = "predictor",
    hdrs = ["predictor.h"],
    srcs = ["predictor.cc"],
    deps = [
        "//modules/prediction/proto:prediction_proto",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/common/proto:path_point_proto",
    ]
)

cpplint()