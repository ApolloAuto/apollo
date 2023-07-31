load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ])
)

cyber_plugin_description(
    name = "plugin_model_description",
    description = ":plugins.xml",
    plugin = ":libmodel.so",
)

cc_binary(
    name = "libmodel.so",
    linkshared = True,
    linkstatic = True,
    deps = [":model_lib"],
)

cc_library(
    name = "model_lib",
    srcs = ["semantic_lstm_pedestrian_torch_model.cc"],
    hdrs = ["semantic_lstm_pedestrian_torch_model.h"],
    deps = [
        "//cyber",
        "//modules/prediction/proto:prediction_conf_cc_proto",
        "//modules/prediction/evaluator/warm_up:warm_up",
        "//modules/prediction/evaluator/model_manager/model:model_base",
        "@libtorch_gpu",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
