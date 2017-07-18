load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "predictor_factory",
    hdrs = ["predictor_factory.h"],
    srcs = ["predictor_factory.cc"],
    deps = [
        "//modules/prediction/predictor:predictor",
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
        "//modules/perception/proto:perception_proto",
        "//modules/common:macro",
        "@glog//:glog"
    ]
)

cc_library(
    name = "predictor",
    hdrs = ["predictor.h"],
    deps = [
    
    ]
)

cpplint()