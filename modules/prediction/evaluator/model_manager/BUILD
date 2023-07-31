load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "model_manager",
    srcs = ["model_manager.cc"],
    hdrs = ["model_manager.h"],
    deps = [
        "//cyber",
        "//modules/prediction/common:prediction_system_gflags",
        "//modules/prediction/proto:prediction_conf_cc_proto",
        "//modules/prediction/evaluator/warm_up",
        "//modules/prediction/evaluator/model_manager/model:model_base",
    ],
)

apollo_package()
cpplint()
