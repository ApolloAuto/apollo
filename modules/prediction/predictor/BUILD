load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "predictor_factory",
    srcs = ["predictor_factory.cc"],
    hdrs = ["predictor_factory.h"],
    deps = [
        "//modules/common:macro",
        "//modules/common/util:factory",
        "//modules/prediction/predictor",
        "//modules/prediction/predictor/vehicle:free_move_predictor",
        "//modules/prediction/predictor/vehicle:lane_sequence_predictor",
        "//modules/prediction/proto:prediction_conf_proto",
        "@glog//:glog",
    ],
)

cc_library(
    name = "predictor_manager",
    srcs = ["predictor_manager.cc"],
    hdrs = ["predictor_manager.h"],
    deps = [
        "//modules/common:macro",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/predictor",
        "//modules/prediction/predictor:predictor_factory",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/proto:prediction_proto",
        "@glog//:glog",
    ],
)

cc_library(
    name = "predictor",
    srcs = ["predictor.cc"],
    hdrs = ["predictor.h"],
    deps = [
        "//modules/common/proto:path_point_proto",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cpplint()
