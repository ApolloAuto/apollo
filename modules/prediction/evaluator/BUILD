package(default_visibility = ["//visibility:public"])

cc_library(
    name = "evaluator_factory",
    hdrs = ["evaluator_factory.h"],
    deps = [
        "//modules/prediction/evaluator:evaluator",
        "//modules/common/util:factory",
        "//modules/common:macro",
    ]
)

cc_library(
    name = "evaluator_manager",
    srcs = ["evaluator_manager.cc"],
    hdrs = ["evaluator_manager.h"],
    deps = [
        "//modules/prediction/evaluator:evaluator",
        "//modules/perception/proto:perception_proto",
        "//modules/common:macro",
        "@glog//:glog",
    ]
)

cc_library(
    name = "evaluator",
    hdrs = ["evaluator.h"],
    deps = [
    
    ]
)