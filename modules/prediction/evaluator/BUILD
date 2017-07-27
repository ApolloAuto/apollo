load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "evaluator_factory",
    srcs = ["evaluator_factory.cc"],
    hdrs = ["evaluator_factory.h"],
    deps = [
        "//modules/common:macro",
        "//modules/common/util:factory",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/proto:prediction_conf_proto",
        "@glog//:glog",
    ],
)

cc_library(
    name = "evaluator_manager",
    srcs = ["evaluator_manager.cc"],
    hdrs = ["evaluator_manager.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/perception/proto:perception_proto",
        "@glog//:glog",
    ],
)

cc_library(
    name = "evaluator",
    hdrs = ["evaluator.h"],
    deps = [
        "//modules/prediction/container/obstacles:obstacle",
    ],
)

cpplint()
