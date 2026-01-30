load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ])
)

apollo_plugin(
    name = "libmodel.so",
    description = ":plugins.xml",
    srcs = ["semantic_lstm_pedestrian_tensorrt.cc"],
    hdrs = ["semantic_lstm_pedestrian_tensorrt.h"],
    deps = [
        "//cyber",
        "//modules/prediction/proto:prediction_conf_cc_proto",
        "//modules/prediction:apollo_prediction",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

apollo_package(enable_source=False)
cpplint()
